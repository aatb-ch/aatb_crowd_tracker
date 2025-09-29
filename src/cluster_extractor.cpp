#include "aatb_crowd_tracker/cluster_extractor.hpp"
#include <cmath>
#include <algorithm>

namespace aatb_crowd_tracker {

ClusterExtractor::ClusterExtractor(size_t min_cluster_size,
                                 size_t max_cluster_size,
                                 double base_distance_threshold,
                                 double distance_scaling_factor)
    : min_cluster_size_(min_cluster_size),
      max_cluster_size_(max_cluster_size),
      base_distance_threshold_(base_distance_threshold),
      distance_scaling_factor_(distance_scaling_factor) {
}

std::vector<Cluster> ClusterExtractor::extractClusters(
    const sensor_msgs::msg::LaserScan& scan,
    const BackgroundModel& background_model) {
    
    std::vector<Cluster> clusters;
    
    // Get valid points (non-background, valid readings)
    auto valid_points = getValidPoints(scan, background_model);
    
    if (valid_points.empty()) {
        return clusters;
    }
    
    // Current cluster being built
    std::vector<geometry_msgs::msg::Point> current_cluster;
    geometry_msgs::msg::Point previous_point = valid_points[0].first;
    current_cluster.push_back(previous_point);
    
    // Iterate through consecutive valid points
    for (size_t i = 1; i < valid_points.size(); ++i) {
        const auto& current_point = valid_points[i].first;
        double dist = distance(previous_point, current_point);
        
        // Calculate adaptive threshold based on distance from sensor
        double range_to_sensor = std::sqrt(current_point.x * current_point.x + 
                                         current_point.y * current_point.y);
        double threshold = adaptiveThreshold(range_to_sensor);
        
        if (dist < threshold) {
            // Point belongs to current cluster
            current_cluster.push_back(current_point);
            
            // Check if cluster is getting too large
            if (current_cluster.size() > max_cluster_size_) {
                // Finalize current cluster and start a new one
                if (current_cluster.size() >= min_cluster_size_) {
                    Cluster cluster;
                    cluster.points = current_cluster;
                    cluster.centroid = calculateCentroid(current_cluster);
                    cluster.point_count = current_cluster.size();
                    clusters.push_back(cluster);
                }
                current_cluster.clear();
                current_cluster.push_back(current_point);
            }
        } else {
            // Gap too large, finalize current cluster if valid
            if (current_cluster.size() >= min_cluster_size_) {
                Cluster cluster;
                cluster.points = current_cluster;
                cluster.centroid = calculateCentroid(current_cluster);
                cluster.point_count = current_cluster.size();
                clusters.push_back(cluster);
            }
            
            // Start new cluster
            current_cluster.clear();
            current_cluster.push_back(current_point);
        }
        
        previous_point = current_point;
    }
    
    // Handle the last cluster
    if (current_cluster.size() >= min_cluster_size_) {
        Cluster cluster;
        cluster.points = current_cluster;
        cluster.centroid = calculateCentroid(current_cluster);
        cluster.point_count = current_cluster.size();
        clusters.push_back(cluster);
    }
    
    return clusters;
}

void ClusterExtractor::setParameters(size_t min_size, size_t max_size,
                                    double base_threshold, double scaling_factor) {
    min_cluster_size_ = min_size;
    max_cluster_size_ = max_size;
    base_distance_threshold_ = base_threshold;
    distance_scaling_factor_ = scaling_factor;
}

geometry_msgs::msg::Point ClusterExtractor::polarToCartesian(
    size_t scan_idx, double range, const sensor_msgs::msg::LaserScan& scan) const {
    
    geometry_msgs::msg::Point point;
    double angle = scan.angle_min + scan_idx * scan.angle_increment;
    
    point.x = range * std::cos(angle);
    point.y = range * std::sin(angle);
    point.z = 0.0;  // 2D laser scan
    
    return point;
}

double ClusterExtractor::distance(const geometry_msgs::msg::Point& p1,
                                const geometry_msgs::msg::Point& p2) const {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

double ClusterExtractor::adaptiveThreshold(double range) const {
    return base_distance_threshold_ * (1.0 + distance_scaling_factor_ * range);
}

geometry_msgs::msg::Point ClusterExtractor::calculateCentroid(
    const std::vector<geometry_msgs::msg::Point>& points) const {
    
    geometry_msgs::msg::Point centroid;
    centroid.x = 0.0;
    centroid.y = 0.0;
    centroid.z = 0.0;
    
    if (points.empty()) {
        return centroid;
    }
    
    for (const auto& point : points) {
        centroid.x += point.x;
        centroid.y += point.y;
    }
    
    centroid.x /= static_cast<double>(points.size());
    centroid.y /= static_cast<double>(points.size());
    
    return centroid;
}

std::vector<std::pair<geometry_msgs::msg::Point, size_t>> 
ClusterExtractor::getValidPoints(const sensor_msgs::msg::LaserScan& scan,
                                const BackgroundModel& background_model) const {
    
    std::vector<std::pair<geometry_msgs::msg::Point, size_t>> valid_points;
    
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        double range = scan.ranges[i];
        
        // Skip invalid readings
        if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
            continue;
        }
        
        // Skip background points
        if (background_model.isBackground(i, range, scan)) {
            continue;
        }
        
        // Convert to Cartesian and add to valid points
        geometry_msgs::msg::Point point = polarToCartesian(i, range, scan);
        valid_points.emplace_back(point, i);
    }
    
    return valid_points;
}

} // namespace aatb_crowd_tracker