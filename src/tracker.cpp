#include "aatb_crowd_tracker/tracker.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

namespace aatb_crowd_tracker {

Tracker::Tracker(rclcpp::Node* node,
               double association_threshold,
               uint32_t max_consecutive_misses,
               double min_track_confidence,
               double movement_threshold)
    : node_(node),
      next_track_id_(1),
      association_threshold_(association_threshold),
      max_consecutive_misses_(max_consecutive_misses),
      min_track_confidence_(min_track_confidence),
      movement_threshold_(movement_threshold) {
    
    // Initialize last update time to current time
    last_update_time_ = node_->now();
}

aatb_msgs::msg::Tracks Tracker::updateTracks(
    const std::vector<Cluster>& clusters,
    const builtin_interfaces::msg::Time& current_time) {
    
    // Calculate time step
    double dt = timeToSeconds(current_time) - timeToSeconds(last_update_time_);
    if (dt > 0.0) {
        predictTracks(dt);
    }
    
    // Associate clusters with existing tracks
    auto associations = associateClusters(clusters);
    
    // Update existing tracks with associated clusters
    for (const auto& [track_id, cluster_idx] : associations) {
        if (cluster_idx >= 0) {
            updateTrack(track_id, clusters[cluster_idx], current_time);
        }
    }
    
    // Create new tracks for unassociated clusters
    for (size_t i = 0; i < clusters.size(); ++i) {
        bool is_associated = false;
        for (const auto& [track_id, cluster_idx] : associations) {
            if (cluster_idx == static_cast<int>(i)) {
                is_associated = true;
                break;
            }
        }
        
        if (!is_associated) {
            createTrack(clusters[i], current_time);
        }
    }
    
    // Mark tracks that weren't updated as missed
    for (auto& [track_id, track] : tracks_) {
        if (associations.find(track_id) == associations.end() || 
            associations[track_id] < 0) {
            track.consecutive_misses++;
        }
    }
    
    // Remove stale tracks
    pruneStaleTrackers(current_time);
    
    // Build tracks message
    aatb_msgs::msg::Tracks tracks_msg;
    tracks_msg.header.stamp = current_time;
    tracks_msg.header.frame_id = "world";  // Assuming world frame
    
    for (const auto& [track_id, track_state] : tracks_) {
        if (track_state.confidence >= min_track_confidence_) {
            tracks_msg.tracks.push_back(trackStateToMessage(track_state, current_time));
        }
    }
    
    last_update_time_ = current_time;
    return tracks_msg;
}

void Tracker::setParameters(double association_threshold,
                          uint32_t max_misses,
                          double min_confidence,
                          double movement_threshold) {
    association_threshold_ = association_threshold;
    max_consecutive_misses_ = max_misses;
    min_track_confidence_ = min_confidence;
    movement_threshold_ = movement_threshold;
}

void Tracker::clearTracks() {
    tracks_.clear();
    next_track_id_ = 1;
}

std::unordered_map<uint32_t, int> Tracker::associateClusters(
    const std::vector<Cluster>& clusters) {
    
    std::unordered_map<uint32_t, int> associations;
    
    // Initialize all tracks as unassociated
    for (const auto& [track_id, track] : tracks_) {
        associations[track_id] = -1;  // -1 means no associated cluster
    }
    
    if (clusters.empty()) {
        return associations;
    }
    
    // Simple nearest neighbor association
    // For each track, find the closest cluster within threshold
    for (auto& [track_id, track] : tracks_) {
        double min_distance = std::numeric_limits<double>::max();
        int best_cluster_idx = -1;
        
        for (size_t i = 0; i < clusters.size(); ++i) {
            double dist = distance(track.position, clusters[i].centroid);
            if (dist < association_threshold_ && dist < min_distance) {
                // Check if this cluster is already associated with a closer track
                bool already_associated = false;
                for (const auto& [other_id, other_idx] : associations) {
                    if (other_idx == static_cast<int>(i)) {
                        // Get distance of other track to this cluster
                        double other_dist = distance(tracks_[other_id].position, clusters[i].centroid);
                        if (other_dist < dist) {
                            already_associated = true;
                            break;
                        } else {
                            // Current track is closer, remove previous association
                            associations[other_id] = -1;
                        }
                    }
                }
                
                if (!already_associated) {
                    min_distance = dist;
                    best_cluster_idx = static_cast<int>(i);
                }
            }
        }
        
        associations[track_id] = best_cluster_idx;
    }
    
    return associations;
}

uint32_t Tracker::createTrack(const Cluster& cluster,
                             const builtin_interfaces::msg::Time& current_time) {
    uint32_t track_id = next_track_id_++;
    
    TrackState track;
    track.id = track_id;
    track.position = cluster.centroid;
    track.velocity.x = 0.0;
    track.velocity.y = 0.0;
    track.velocity.z = 0.0;
    track.first_detected = current_time;
    track.last_updated = current_time;
    track.consecutive_misses = 0;
    track.is_moving = false;
    track.confidence = 1.0;  // Full confidence for new track
    
    tracks_[track_id] = track;
    return track_id;
}

void Tracker::updateTrack(uint32_t track_id, const Cluster& cluster,
                         const builtin_interfaces::msg::Time& current_time) {
    auto it = tracks_.find(track_id);
    if (it == tracks_.end()) {
        return;
    }
    
    TrackState& track = it->second;
    
    // Calculate time step
    double dt = timeToSeconds(current_time) - timeToSeconds(track.last_updated);
    
    if (dt > 0.0) {
        // Update velocity estimate
        track.velocity = calculateVelocity(track.position, cluster.centroid, dt);
        
        // Update movement detection
        double speed = std::sqrt(track.velocity.x * track.velocity.x + 
                               track.velocity.y * track.velocity.y);
        track.is_moving = (speed > movement_threshold_);
    }
    
    // Update position (simple update, could use Kalman filter here)
    track.position = cluster.centroid;
    track.last_updated = current_time;
    track.consecutive_misses = 0;
    
    // Update confidence (increase for successful updates)
    track.confidence = std::min(1.0, track.confidence + 0.1);
}

void Tracker::predictTracks(double dt) {
    for (auto& [track_id, track] : tracks_) {
        // Simple constant velocity prediction
        track.position.x += track.velocity.x * dt;
        track.position.y += track.velocity.y * dt;
        
        // Decrease confidence for predictions
        track.confidence *= 0.95;
    }
}

void Tracker::pruneStaleTrackers(const builtin_interfaces::msg::Time& current_time) {
    auto it = tracks_.begin();
    while (it != tracks_.end()) {
        if (it->second.consecutive_misses > max_consecutive_misses_) {
            it = tracks_.erase(it);
        } else {
            ++it;
        }
    }
}

double Tracker::distance(const geometry_msgs::msg::Point& p1,
                        const geometry_msgs::msg::Point& p2) const {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

geometry_msgs::msg::Vector3 Tracker::calculateVelocity(
    const geometry_msgs::msg::Point& old_pos,
    const geometry_msgs::msg::Point& new_pos,
    double dt) const {
    
    geometry_msgs::msg::Vector3 velocity;
    
    if (dt > 0.0) {
        velocity.x = (new_pos.x - old_pos.x) / dt;
        velocity.y = (new_pos.y - old_pos.y) / dt;
        velocity.z = 0.0;  // 2D tracking
    } else {
        velocity.x = 0.0;
        velocity.y = 0.0;
        velocity.z = 0.0;
    }
    
    return velocity;
}

double Tracker::timeToSeconds(const builtin_interfaces::msg::Time& time_msg) const {
    return static_cast<double>(time_msg.sec) + static_cast<double>(time_msg.nanosec) * 1e-9;
}

builtin_interfaces::msg::Duration Tracker::calculateDuration(
    const builtin_interfaces::msg::Time& start_time,
    const builtin_interfaces::msg::Time& end_time) const {
    
    builtin_interfaces::msg::Duration duration;
    
    double start_sec = timeToSeconds(start_time);
    double end_sec = timeToSeconds(end_time);
    double duration_sec = end_sec - start_sec;
    
    duration.sec = static_cast<int32_t>(duration_sec);
    duration.nanosec = static_cast<uint32_t>((duration_sec - duration.sec) * 1e9);
    
    return duration;
}

aatb_msgs::msg::Track Tracker::trackStateToMessage(
    const TrackState& track_state,
    const builtin_interfaces::msg::Time& current_time) const {
    
    aatb_msgs::msg::Track track_msg;
    
    track_msg.id = track_state.id;
    track_msg.entered = track_state.first_detected;
    track_msg.age = calculateDuration(track_state.first_detected, current_time);
    track_msg.moving = track_state.is_moving;
    
    // Set pose
    track_msg.pose.position = track_state.position;
    track_msg.pose.orientation.x = 0.0;
    track_msg.pose.orientation.y = 0.0;
    track_msg.pose.orientation.z = 0.0;
    track_msg.pose.orientation.w = 1.0;  // No rotation
    
    track_msg.velocity = track_state.velocity;
    
    return track_msg;
}

} // namespace aatb_crowd_tracker