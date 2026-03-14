#include "aatb_crowd_tracker/heatmap_model.hpp"
#include <cmath>
#include <algorithm>
#include <queue>

namespace aatb_crowd_tracker {

HeatmapModel::HeatmapModel(const Config& cfg)
    : cfg_(cfg)
    , bins_(cfg.width * cfg.height)
    , hit_mask_(cfg.width * cfg.height, false)
{}

bool HeatmapModel::worldToCell(float x, float y, int& cx, int& cy) const {
    cx = static_cast<int>((x - cfg_.origin_x) / cfg_.resolution);
    cy = static_cast<int>((y - cfg_.origin_y) / cfg_.resolution);
    return cx >= 0 && cx < cfg_.width && cy >= 0 && cy < cfg_.height;
}

void HeatmapModel::runSchmittTrigger(HeatmapBin& bin) {
    const float diff   = bin.fast_avg - bin.slow_avg;
    const float d_diff = diff - bin.prev_diff;

    if (!bin.active) {
        // Activate only on a significant rising edge
        if (diff > cfg_.schmitt_high && d_diff > cfg_.derivative_gate) {
            bin.active = true;
        }
    } else {
        // Deactivate only when fully settled near zero on both value and derivative
        // Stays active through rising or falling edges (|d_diff| >= gate)
        if (std::abs(diff) < cfg_.schmitt_low && std::abs(d_diff) < cfg_.derivative_gate) {
            bin.active = false;
        }
    }

    bin.prev_diff = diff;
}

void HeatmapModel::updateFromPoints(const std::vector<geometry_msgs::msg::Point>& points) {
    std::fill(hit_mask_.begin(), hit_mask_.end(), false);

    for (const auto& pt : points) {
        int cx, cy;
        if (worldToCell(static_cast<float>(pt.x), static_cast<float>(pt.y), cx, cy)) {
            hit_mask_[cy * cfg_.width + cx] = true;
        }
    }

    const int n = cfg_.width * cfg_.height;
    for (int i = 0; i < n; ++i) {
        HeatmapBin& bin = bins_[i];
        if (hit_mask_[i]) {
            bin.fast_avg += cfg_.fast_learning_rate * (1.0f - bin.fast_avg);
            bin.slow_avg += cfg_.slow_learning_rate * (1.0f - bin.slow_avg);
        } else {
            bin.fast_avg *= (1.0f - cfg_.fast_decay_rate);
            bin.slow_avg *= (1.0f - cfg_.slow_decay_rate);
        }
        runSchmittTrigger(bin);
    }
}

void HeatmapModel::decayUpdate() {
    for (auto& bin : bins_) {
        bin.fast_avg *= (1.0f - cfg_.fast_decay_rate);
        bin.slow_avg *= (1.0f - cfg_.slow_decay_rate);
        runSchmittTrigger(bin);
    }
}

nav_msgs::msg::OccupancyGrid HeatmapModel::makeBaseGrid(
    const std_msgs::msg::Header& header) const
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.header = header;
    grid.info.resolution = cfg_.resolution;
    grid.info.width      = static_cast<uint32_t>(cfg_.width);
    grid.info.height     = static_cast<uint32_t>(cfg_.height);
    grid.info.origin.position.x  = cfg_.origin_x;
    grid.info.origin.position.y  = cfg_.origin_y;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(bins_.size());
    return grid;
}

nav_msgs::msg::OccupancyGrid HeatmapModel::buildHeatmapGrid(
    const std_msgs::msg::Header& header) const
{
    auto grid = makeBaseGrid(header);
    for (size_t i = 0; i < bins_.size(); ++i) {
        const float diff       = bins_[i].fast_avg - bins_[i].slow_avg;
        const float normalized = diff / cfg_.schmitt_high;  // 1.0 at full activation
        const int   val        = static_cast<int>(std::round(normalized * 100.0f));
        grid.data[i] = static_cast<int8_t>(std::clamp(val, 0, 100));
    }
    return grid;
}

nav_msgs::msg::OccupancyGrid HeatmapModel::buildActiveGrid(
    const std_msgs::msg::Header& header) const
{
    auto grid = makeBaseGrid(header);
    for (size_t i = 0; i < bins_.size(); ++i) {
        grid.data[i] = bins_[i].active ? 100 : 0;
    }
    return grid;
}

std::vector<Cluster> HeatmapModel::extractBlobs() const {
    std::vector<bool>    visited(bins_.size(), false);
    std::vector<Cluster> blobs;

    for (int row = 0; row < cfg_.height; ++row) {
        for (int col = 0; col < cfg_.width; ++col) {
            const int idx = row * cfg_.width + col;
            if (!bins_[idx].active || visited[idx]) continue;

            // BFS over 8-connected active cells
            std::vector<geometry_msgs::msg::Point> cells;
            std::queue<int> queue;
            queue.push(idx);
            visited[idx] = true;

            while (!queue.empty()) {
                const int cur = queue.front();
                queue.pop();

                const int cx = cur % cfg_.width;
                const int cy = cur / cfg_.width;

                geometry_msgs::msg::Point pt;
                pt.x = cfg_.origin_x + (cx + 0.5f) * cfg_.resolution;
                pt.y = cfg_.origin_y + (cy + 0.5f) * cfg_.resolution;
                pt.z = 0.0;
                cells.push_back(pt);

                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        const int nx = cx + dx;
                        const int ny = cy + dy;
                        if (nx < 0 || nx >= cfg_.width || ny < 0 || ny >= cfg_.height) continue;
                        const int nidx = ny * cfg_.width + nx;
                        if (!bins_[nidx].active || visited[nidx]) continue;
                        visited[nidx] = true;
                        queue.push(nidx);
                    }
                }
            }

            if (static_cast<int>(cells.size()) < cfg_.min_blob_cells ||
                static_cast<int>(cells.size()) > cfg_.max_blob_cells) {
                continue;
            }

            Cluster blob;
            blob.points      = cells;
            blob.point_count = cells.size();

            float sx = 0.0f, sy = 0.0f;
            for (const auto& p : cells) {
                sx += static_cast<float>(p.x);
                sy += static_cast<float>(p.y);
            }
            blob.centroid.x = sx / static_cast<float>(cells.size());
            blob.centroid.y = sy / static_cast<float>(cells.size());
            blob.centroid.z = 0.0;

            blobs.push_back(std::move(blob));
        }
    }

    return blobs;
}

} // namespace aatb_crowd_tracker
