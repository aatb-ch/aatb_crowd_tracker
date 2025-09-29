#include "aatb_crowd_tracker/background_model.hpp"
#include <cmath>

namespace aatb_crowd_tracker {

BackgroundModel::BackgroundModel(float learning_rate,
                               float occupancy_threshold,
                               uint32_t decay_frames,
                               float decay_rate)
    : learning_rate_(learning_rate),
      occupancy_threshold_(occupancy_threshold),
      decay_frames_(decay_frames),
      decay_rate_(decay_rate) {
}

void BackgroundModel::update(const sensor_msgs::msg::LaserScan& scan, uint32_t current_frame) {
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float range = scan.ranges[i];
        
        // Skip invalid readings
        if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
            continue;
        }
        
        updatePoint(i, range, current_frame, scan);
    }
}

bool BackgroundModel::isBackground(size_t scan_idx, float range,
                                  const sensor_msgs::msg::LaserScan& scan) const {
    PolarBin bin = quantize(scan_idx, range, scan);
    auto it = background_map_.find(bin.hash());
    return (it != background_map_.end() && it->second.occupancy_score > occupancy_threshold_);
}

void BackgroundModel::clear() {
    background_map_.clear();
}

PolarBin BackgroundModel::quantize(size_t scan_idx, float range,
                                  const sensor_msgs::msg::LaserScan& scan) const {
    PolarBin bin;
    
    // Calculate angle from scan index
    float angle = scan.angle_min + scan_idx * scan.angle_increment;
    
    // Convert to degrees and normalize to [0, 360)
    float angle_deg = angle * 180.0f / M_PI;
    while (angle_deg < 0) angle_deg += 360.0f;
    while (angle_deg >= 360.0f) angle_deg -= 360.0f;
    
    // Quantize to bin index
    bin.angle_idx = static_cast<uint16_t>(angle_deg / 0.72f) % ANGULAR_BINS;
    bin.range_idx = static_cast<uint16_t>(range / RANGE_RESOLUTION);
    
    return bin;
}

void BackgroundModel::updatePoint(size_t scan_idx, float range, uint32_t current_frame,
                                 const sensor_msgs::msg::LaserScan& scan) {
    PolarBin bin = quantize(scan_idx, range, scan);
    auto& data = background_map_[bin.hash()];
    
    // Apply decay if this bin hasn't been updated recently
    if (data.last_update > 0 && current_frame - data.last_update > decay_frames_) {
        data.occupancy_score *= decay_rate_;
        
        // Remove negligible entries to save memory
        if (data.occupancy_score < 0.01f) {
            background_map_.erase(bin.hash());
            return;
        }
    }
    
    // Update with exponential moving average
    data.occupancy_score = learning_rate_ + (1.0f - learning_rate_) * data.occupancy_score;
    
    // Clamp to [0, 1]
    if (data.occupancy_score > 1.0f) {
        data.occupancy_score = 1.0f;
    }
    
    data.last_update = current_frame;
}

} // namespace aatb_crowd_tracker