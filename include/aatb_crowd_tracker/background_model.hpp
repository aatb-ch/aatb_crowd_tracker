#ifndef AATB_CROWD_TRACKER_BACKGROUND_MODEL_HPP
#define AATB_CROWD_TRACKER_BACKGROUND_MODEL_HPP

#include <unordered_map>
#include <cstdint>
#include <cmath>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace aatb_crowd_tracker {

/**
 * @brief Represents a quantized polar bin for background modeling
 */
struct PolarBin {
    uint16_t angle_idx;  ///< Angular bin index (500 bins, 0.72°/bin)
    uint16_t range_idx;  ///< Range bin index (0.1m resolution)
    
    /**
     * @brief Hash function for use in unordered_map
     * @return Hash value combining angle and range indices
     */
    size_t hash() const {
        return (static_cast<size_t>(angle_idx) << 16) | range_idx;
    }
};

/**
 * @brief Data stored for each occupied bin
 */
struct BinData {
    float occupancy_score;   ///< Score from 0.0 (never seen) to 1.0 (always occupied)
    uint32_t last_update;    ///< Frame number of last update
};

/**
 * @brief Background model for filtering static obstacles
 */
class BackgroundModel {
public:
    /**
     * @brief Constructor
     * @param learning_rate Alpha value for exponential moving average
     * @param occupancy_threshold Threshold above which a point is considered background
     * @param decay_frames Number of frames after which to start decaying
     * @param decay_rate Rate of decay for old entries
     */
    BackgroundModel(float learning_rate = 0.01f,
                   float occupancy_threshold = 0.7f,
                   uint32_t decay_frames = 100,
                   float decay_rate = 0.95f);

    /**
     * @brief Update the background model with a new scan
     * @param scan The laser scan message
     * @param current_frame Current frame number
     */
    void update(const sensor_msgs::msg::LaserScan& scan, uint32_t current_frame);

    /**
     * @brief Check if a point is part of the background.
     *        Computes on-the-fly decay so bins that haven't been re-observed
     *        still lose their background status over time.
     */
    bool isBackground(size_t scan_idx, float range,
                     const sensor_msgs::msg::LaserScan& scan) const;

    /**
     * @brief Clear the entire background model (full reset)
     */
    void clear();

    /**
     * @brief Sweep the map and prune bins whose decayed score is negligible.
     *        Call periodically (e.g. every 1000 frames) to reclaim memory.
     */
    void cleanup();

    /**
     * @brief Get the number of active bins in the model
     * @return Number of bins currently stored
     */
    size_t getActiveBinCount() const { return background_map_.size(); }

private:
    /**
     * @brief Quantize a scan point to a polar bin
     * @param scan_idx Index in the scan array
     * @param range Range value at this index
     * @param scan The laser scan message
     * @return Quantized polar bin
     */
    PolarBin quantize(size_t scan_idx, float range, 
                     const sensor_msgs::msg::LaserScan& scan) const;

    /**
     * @brief Update a single point in the background model
     * @param scan_idx Index in the scan array
     * @param range Range value at this index
     * @param current_frame Current frame number
     * @param scan The laser scan message
     */
    void updatePoint(size_t scan_idx, float range, uint32_t current_frame,
                    const sensor_msgs::msg::LaserScan& scan);

    /// Compute the decayed score of a bin given elapsed frames since last update
    float decayedScore(float score, uint32_t last_update) const {
        if (current_frame_ <= last_update) return score;
        uint32_t elapsed = current_frame_ - last_update;
        if (elapsed <= decay_frames_) return score;
        uint32_t decay_steps = elapsed - decay_frames_;
        return score * std::pow(decay_rate_, static_cast<float>(decay_steps));
    }

    std::unordered_map<size_t, BinData> background_map_;  ///< Sparse storage of occupied bins
    float learning_rate_;                                  ///< Learning rate for exponential average
    float occupancy_threshold_;                           ///< Threshold for background classification
    uint32_t decay_frames_;                               ///< Frames before decay starts
    float decay_rate_;                                    ///< Rate of decay for old entries
    uint32_t current_frame_ = 0;                          ///< Latest frame seen by update()

    static constexpr uint16_t ANGULAR_BINS = 500;         ///< Number of angular bins (0.72°/bin)
    static constexpr float RANGE_RESOLUTION = 0.5f;       ///< Range quantization in meters
};

} // namespace aatb_crowd_tracker

#endif // AATB_CROWD_TRACKER_BACKGROUND_MODEL_HPP