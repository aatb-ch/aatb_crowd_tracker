#ifndef AATB_CROWD_TRACKER_TRACKER_HPP
#define AATB_CROWD_TRACKER_TRACKER_HPP

#include <vector>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <aatb_msgs/msg/track.hpp>
#include <aatb_msgs/msg/tracks.hpp>
#include "aatb_crowd_tracker/cluster_extractor.hpp"

namespace aatb_crowd_tracker {

/**
 * @brief Internal track state for tracking algorithms
 */
struct TrackState {
    uint32_t id;                                      ///< Unique track ID
    geometry_msgs::msg::Point position;               ///< Current position (world frame)
    geometry_msgs::msg::Vector3 velocity;             ///< Current velocity estimate
    builtin_interfaces::msg::Time first_detected;     ///< Time when first detected
    builtin_interfaces::msg::Time last_updated;       ///< Time of last update
    uint32_t consecutive_misses;                      ///< Number of consecutive frames without update
    bool is_moving;                                   ///< Movement detection flag
    double confidence;                                ///< Track confidence score
};

/**
 * @brief Tracks clusters over time and manages track lifecycle
 */
class Tracker {
public:
    /**
     * @brief Constructor
     * @param node ROS2 node for time access
     * @param association_threshold Maximum distance for track-cluster association
     * @param max_consecutive_misses Maximum misses before track deletion
     * @param min_track_confidence Minimum confidence for track publication
     * @param movement_threshold Minimum velocity to consider track as moving
     */
    Tracker(rclcpp::Node* node,
           double association_threshold = 1.0,
           uint32_t max_consecutive_misses = 10,
           double min_track_confidence = 0.5,
           double movement_threshold = 0.1);

    /**
     * @brief Update tracks with new cluster observations
     * @param clusters Vector of clusters from current scan (world frame)
     * @param current_time Current timestamp
     * @return Updated tracks message
     */
    aatb_msgs::msg::Tracks updateTracks(const std::vector<Cluster>& clusters,
                                       const builtin_interfaces::msg::Time& current_time);

    /**
     * @brief Set tracking parameters
     * @param association_threshold Maximum association distance
     * @param max_misses Maximum consecutive misses
     * @param min_confidence Minimum track confidence
     * @param movement_threshold Movement detection threshold
     */
    void setParameters(double association_threshold,
                      uint32_t max_misses,
                      double min_confidence,
                      double movement_threshold);

    /**
     * @brief Clear all tracks
     */
    void clearTracks();

    /**
     * @brief Get the number of active tracks
     * @return Number of currently tracked objects
     */
    size_t getActiveTrackCount() const { return tracks_.size(); }

private:
    /**
     * @brief Associate clusters with existing tracks using nearest neighbor
     * @param clusters Clusters to associate
     * @return Map of track ID to cluster index, unassociated clusters get -1
     */
    std::unordered_map<uint32_t, int> associateClusters(const std::vector<Cluster>& clusters);

    /**
     * @brief Create a new track from a cluster
     * @param cluster Cluster to create track from
     * @param current_time Current timestamp
     * @return New track ID
     */
    uint32_t createTrack(const Cluster& cluster, 
                        const builtin_interfaces::msg::Time& current_time);

    /**
     * @brief Update an existing track with a cluster observation
     * @param track_id Track to update
     * @param cluster Cluster observation
     * @param current_time Current timestamp
     */
    void updateTrack(uint32_t track_id, const Cluster& cluster,
                    const builtin_interfaces::msg::Time& current_time);

    /**
     * @brief Predict track positions based on velocity (simple constant velocity model)
     * @param dt Time step in seconds
     */
    void predictTracks(double dt);

    /**
     * @brief Remove stale tracks that haven't been updated
     * @param current_time Current timestamp
     */
    void pruneStaleTrackers(const builtin_interfaces::msg::Time& current_time);

    /**
     * @brief Calculate distance between two points
     * @param p1 First point
     * @param p2 Second point
     * @return Euclidean distance
     */
    double distance(const geometry_msgs::msg::Point& p1,
                   const geometry_msgs::msg::Point& p2) const;

    /**
     * @brief Calculate velocity from position change
     * @param old_pos Previous position
     * @param new_pos Current position
     * @param dt Time step in seconds
     * @return Velocity vector
     */
    geometry_msgs::msg::Vector3 calculateVelocity(
        const geometry_msgs::msg::Point& old_pos,
        const geometry_msgs::msg::Point& new_pos,
        double dt) const;

    /**
     * @brief Convert time message to seconds
     * @param time_msg Time message
     * @return Time in seconds
     */
    double timeToSeconds(const builtin_interfaces::msg::Time& time_msg) const;

    /**
     * @brief Calculate duration between two timestamps
     * @param start_time Start timestamp
     * @param end_time End timestamp
     * @return Duration message
     */
    builtin_interfaces::msg::Duration calculateDuration(
        const builtin_interfaces::msg::Time& start_time,
        const builtin_interfaces::msg::Time& end_time) const;

    /**
     * @brief Convert internal track state to published Track message
     * @param track_state Internal track state
     * @param current_time Current timestamp
     * @return Track message ready for publication
     */
    aatb_msgs::msg::Track trackStateToMessage(
        const TrackState& track_state,
        const builtin_interfaces::msg::Time& current_time) const;

    rclcpp::Node* node_;                             ///< ROS2 node for time access
    std::unordered_map<uint32_t, TrackState> tracks_; ///< Active tracks indexed by ID
    uint32_t next_track_id_;                          ///< Counter for assigning unique track IDs
    
    // Parameters
    double association_threshold_;                    ///< Maximum distance for track association
    uint32_t max_consecutive_misses_;                ///< Maximum misses before deletion
    double min_track_confidence_;                    ///< Minimum confidence for publication
    double movement_threshold_;                      ///< Minimum velocity for movement detection
    
    builtin_interfaces::msg::Time last_update_time_; ///< Time of last update for dt calculation
};

} // namespace aatb_crowd_tracker

#endif // AATB_CROWD_TRACKER_TRACKER_HPP