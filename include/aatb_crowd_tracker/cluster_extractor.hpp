#ifndef AATB_CROWD_TRACKER_CLUSTER_EXTRACTOR_HPP
#define AATB_CROWD_TRACKER_CLUSTER_EXTRACTOR_HPP

#include <vector>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "aatb_crowd_tracker/background_model.hpp"

namespace aatb_crowd_tracker {

/**
 * @brief Represents a cluster of laser scan points
 */
struct Cluster {
    std::vector<geometry_msgs::msg::Point> points;  ///< Points in the cluster (laser frame)
    geometry_msgs::msg::Point centroid;             ///< Cluster centroid (laser frame)
    size_t point_count;                             ///< Number of points in cluster
};

/**
 * @brief Extracts clusters from laser scan data after background filtering
 */
class ClusterExtractor {
public:
    /**
     * @brief Constructor
     * @param min_cluster_size Minimum number of points for a valid cluster
     * @param max_cluster_size Maximum number of points for a valid cluster
     * @param base_distance_threshold Base threshold for clustering adjacent points
     * @param distance_scaling_factor Factor to scale threshold based on distance from sensor
     */
    ClusterExtractor(size_t min_cluster_size = 3,
                    size_t max_cluster_size = 100,
                    double base_distance_threshold = 0.3,
                    double distance_scaling_factor = 0.1);

    /**
     * @brief Extract clusters from a laser scan
     * @param scan The laser scan message
     * @param background_model Background model for filtering static points
     * @return Vector of extracted clusters
     */
    std::vector<Cluster> extractClusters(const sensor_msgs::msg::LaserScan& scan,
                                        const BackgroundModel& background_model);

    /**
     * @brief Set clustering parameters
     * @param min_size Minimum cluster size
     * @param max_size Maximum cluster size
     * @param base_threshold Base distance threshold
     * @param scaling_factor Distance scaling factor
     */
    void setParameters(size_t min_size, size_t max_size,
                      double base_threshold, double scaling_factor);

private:
    /**
     * @brief Convert polar scan point to Cartesian coordinates
     * @param scan_idx Index in the scan array
     * @param range Range value
     * @param scan The laser scan message
     * @return Point in Cartesian coordinates (laser frame)
     */
    geometry_msgs::msg::Point polarToCartesian(size_t scan_idx, double range,
                                              const sensor_msgs::msg::LaserScan& scan) const;

    /**
     * @brief Calculate distance between two points
     * @param p1 First point
     * @param p2 Second point
     * @return Euclidean distance
     */
    double distance(const geometry_msgs::msg::Point& p1,
                   const geometry_msgs::msg::Point& p2) const;

    /**
     * @brief Calculate adaptive distance threshold based on distance from sensor
     * @param range Distance from sensor
     * @return Adaptive threshold
     */
    double adaptiveThreshold(double range) const;

    /**
     * @brief Calculate centroid of a cluster
     * @param points Vector of points in the cluster
     * @return Centroid point
     */
    geometry_msgs::msg::Point calculateCentroid(const std::vector<geometry_msgs::msg::Point>& points) const;

    /**
     * @brief Filter valid points from laser scan (remove background and invalid readings)
     * @param scan The laser scan message
     * @param background_model Background model for filtering
     * @return Vector of valid points with their scan indices
     */
    std::vector<std::pair<geometry_msgs::msg::Point, size_t>> getValidPoints(
        const sensor_msgs::msg::LaserScan& scan,
        const BackgroundModel& background_model) const;

    size_t min_cluster_size_;           ///< Minimum points per cluster
    size_t max_cluster_size_;           ///< Maximum points per cluster
    double base_distance_threshold_;    ///< Base clustering threshold
    double distance_scaling_factor_;    ///< Scaling factor for distance-based threshold
};

} // namespace aatb_crowd_tracker

#endif // AATB_CROWD_TRACKER_CLUSTER_EXTRACTOR_HPP