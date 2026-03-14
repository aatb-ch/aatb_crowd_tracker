#ifndef AATB_CROWD_TRACKER_HEATMAP_MODEL_HPP
#define AATB_CROWD_TRACKER_HEATMAP_MODEL_HPP

#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/header.hpp>
#include "aatb_crowd_tracker/cluster_extractor.hpp"

namespace aatb_crowd_tracker {

struct HeatmapBin {
    float slow_avg  = 0.0f;
    float fast_avg  = 0.0f;
    bool  active    = false;
    float prev_diff = 0.0f;
};

class HeatmapModel {
public:
    struct Config {
        float resolution         = 0.2f;
        float origin_x           = -15.0f;
        float origin_y           = -15.0f;
        int   width              = 150;
        int   height             = 150;
        float slow_learning_rate = 0.002f;
        float fast_learning_rate = 0.05f;
        float slow_decay_rate    = 0.001f;
        float fast_decay_rate    = 0.02f;
        float schmitt_high       = 0.10f;
        float schmitt_low        = 0.02f;
        float derivative_gate    = 0.005f;
        int   min_blob_cells     = 2;
        int   max_blob_cells     = 500;
    };

    explicit HeatmapModel(const Config& cfg);

    // Update EMAs from scan points (XY in output frame) and run Schmitt trigger
    void updateFromPoints(const std::vector<geometry_msgs::msg::Point>& points);

    // Decay-only update for timeout fallback (no new hits)
    void decayUpdate();

    // Heatmap grid: value = clamp(round((fast-slow)/schmitt_high * 100), 0, 100)
    nav_msgs::msg::OccupancyGrid buildHeatmapGrid(const std_msgs::msg::Header& header) const;

    // Activation grid: 100 if active, 0 otherwise
    nav_msgs::msg::OccupancyGrid buildActiveGrid(const std_msgs::msg::Header& header) const;

    // BFS connected-component blob extraction on the active grid (8-connectivity)
    std::vector<Cluster> extractBlobs() const;

    int cellCount() const { return cfg_.width * cfg_.height; }

private:
    bool worldToCell(float x, float y, int& cx, int& cy) const;
    void runSchmittTrigger(HeatmapBin& bin);
    nav_msgs::msg::OccupancyGrid makeBaseGrid(const std_msgs::msg::Header& header) const;

    Config cfg_;
    std::vector<HeatmapBin> bins_;     // row-major: [row * width + col]
    std::vector<bool>       hit_mask_; // reused each scan update to avoid allocation
};

} // namespace aatb_crowd_tracker

#endif // AATB_CROWD_TRACKER_HEATMAP_MODEL_HPP
