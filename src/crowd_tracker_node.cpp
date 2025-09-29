#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <aatb_msgs/msg/tracks.hpp>

#include "aatb_crowd_tracker/background_model.hpp"
#include "aatb_crowd_tracker/cluster_extractor.hpp"
#include "aatb_crowd_tracker/tracker.hpp"

namespace aatb_crowd_tracker {

/**
 * @brief Main ROS2 node for crowd tracking
 */
class CrowdTrackerNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     */
    CrowdTrackerNode() : Node("crowd_tracker_node"), frame_counter_(0) {
        initializeParameters();
        initializeTransforms();
        initializeComponents();
        initializePubSub();
        
        RCLCPP_INFO(get_logger(), "CrowdTracker node initialized");
    }

private:
    /**
     * @brief Initialize ROS parameters
     */
    void initializeParameters() {
        // Background model parameters
        declare_parameter("background_model.learning_rate", 0.01);
        declare_parameter("background_model.occupancy_threshold", 0.7);
        declare_parameter("background_model.decay_frames", 100);
        declare_parameter("background_model.decay_rate", 0.95);
        declare_parameter("background_model.enable", true);
        
        // Clustering parameters
        declare_parameter("cluster_extractor.min_cluster_size", 3);
        declare_parameter("cluster_extractor.max_cluster_size", 100);
        declare_parameter("cluster_extractor.base_distance_threshold", 0.3);
        declare_parameter("cluster_extractor.distance_scaling_factor", 0.1);
        
        // Tracking parameters
        declare_parameter("tracker.association_threshold", 1.0);
        declare_parameter("tracker.max_consecutive_misses", 10);
        declare_parameter("tracker.min_track_confidence", 0.5);
        declare_parameter("tracker.movement_threshold", 0.1);
        
        // Node parameters
        declare_parameter("scan_topic", "/scan");
        declare_parameter("tracks_topic", "/tracks");
        declare_parameter("posearray_topic", "/tracks_posearray");
        declare_parameter("enable_posearray_debug", true);
        declare_parameter("laser_frame_id", "laser_link");
        declare_parameter("world_frame_id", "world");
        declare_parameter("publish_rate", 10.0);
    }

    /**
     * @brief Initialize transform handling
     */
    void initializeTransforms() {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Get frame IDs
        laser_frame_id_ = get_parameter("laser_frame_id").as_string();
        world_frame_id_ = get_parameter("world_frame_id").as_string();
        
        // Wait for transform to be available (with timeout)
        RCLCPP_INFO(get_logger(), "Waiting for transform from %s to %s...", 
                   laser_frame_id_.c_str(), world_frame_id_.c_str());
        
        // Try to get static transform with retries
        int retry_count = 0;
        const int max_retries = 10;
        
        while (retry_count < max_retries && rclcpp::ok()) {
            try {
                laser_to_world_transform_ = tf_buffer_->lookupTransform(
                    world_frame_id_, laser_frame_id_, tf2::TimePointZero, 
                    std::chrono::seconds(1));
                
                RCLCPP_INFO(get_logger(), "Transform cached successfully");
                transform_available_ = true;
                break;
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(get_logger(), "Transform not available yet: %s", ex.what());
                retry_count++;
                rclcpp::sleep_for(std::chrono::seconds(1));
            }
        }
        
        if (!transform_available_) {
            RCLCPP_WARN(get_logger(), "Could not get transform, will work in laser frame");
        }
    }

    /**
     * @brief Initialize processing components
     */
    void initializeComponents() {
        // Initialize background model
        auto bg_learning_rate = get_parameter("background_model.learning_rate").as_double();
        auto bg_threshold = get_parameter("background_model.occupancy_threshold").as_double();
        auto bg_decay_frames = get_parameter("background_model.decay_frames").as_int();
        auto bg_decay_rate = get_parameter("background_model.decay_rate").as_double();
        
        background_model_ = std::make_unique<BackgroundModel>(
            bg_learning_rate, bg_threshold, bg_decay_frames, bg_decay_rate);
        
        // Initialize cluster extractor
        auto min_cluster = get_parameter("cluster_extractor.min_cluster_size").as_int();
        auto max_cluster = get_parameter("cluster_extractor.max_cluster_size").as_int();
        auto base_threshold = get_parameter("cluster_extractor.base_distance_threshold").as_double();
        auto scaling_factor = get_parameter("cluster_extractor.distance_scaling_factor").as_double();
        
        cluster_extractor_ = std::make_unique<ClusterExtractor>(
            min_cluster, max_cluster, base_threshold, scaling_factor);
        
        // Initialize tracker
        auto assoc_threshold = get_parameter("tracker.association_threshold").as_double();
        auto max_misses = get_parameter("tracker.max_consecutive_misses").as_int();
        auto min_confidence = get_parameter("tracker.min_track_confidence").as_double();
        auto movement_threshold = get_parameter("tracker.movement_threshold").as_double();
        
        tracker_ = std::make_unique<Tracker>(
            this, assoc_threshold, max_misses, min_confidence, movement_threshold);
        
        // Cache parameter values
        enable_background_removal_ = get_parameter("background_model.enable").as_bool();
        enable_posearray_debug_ = get_parameter("enable_posearray_debug").as_bool();
    }

    /**
     * @brief Initialize publishers and subscribers
     */
    void initializePubSub() {
        auto scan_topic = get_parameter("scan_topic").as_string();
        auto tracks_topic = get_parameter("tracks_topic").as_string();
        auto posearray_topic = get_parameter("posearray_topic").as_string();
        
        // Subscribe to laser scan
        laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                processLaserScan(msg);
            });
        
        // Publish tracks
        tracks_pub_ = create_publisher<aatb_msgs::msg::Tracks>(
            tracks_topic, 10);
        
        // Publish PoseArray for debugging if enabled
        if (enable_posearray_debug_) {
            posearray_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
                posearray_topic, 10);
            RCLCPP_INFO(get_logger(), "Subscribed to %s, publishing to %s and %s", 
                       scan_topic.c_str(), tracks_topic.c_str(), posearray_topic.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "Subscribed to %s, publishing to %s", 
                       scan_topic.c_str(), tracks_topic.c_str());
        }
    }

    /**
     * @brief Process incoming laser scan message
     * @param msg Laser scan message
     */
    void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        frame_counter_++;
        
        // Update background model if enabled
        if (enable_background_removal_) {
            background_model_->update(*msg, frame_counter_);
        }
        
        // Extract clusters (background filtering happens inside if enabled)
        std::vector<Cluster> clusters;
        if (enable_background_removal_) {
            clusters = cluster_extractor_->extractClusters(*msg, *background_model_);
        } else {
            // Create dummy background model that considers nothing as background
            BackgroundModel dummy_background(0.0, 2.0, 1000, 1.0);  // Threshold > 1 means nothing is background
            clusters = cluster_extractor_->extractClusters(*msg, dummy_background);
        }
        
        // Transform clusters to world frame if transform is available
        if (transform_available_) {
            transformClustersToWorld(clusters);
        }
        
        // Update tracker
        auto tracks_msg = tracker_->updateTracks(clusters, msg->header.stamp);
        
        // Set frame_id appropriately
        if (transform_available_) {
            tracks_msg.header.frame_id = world_frame_id_;
        } else {
            tracks_msg.header.frame_id = msg->header.frame_id;
        }
        
        // Publish tracks
        tracks_pub_->publish(tracks_msg);
        
        // Publish PoseArray for debugging if enabled
        if (enable_posearray_debug_ && posearray_pub_) {
            auto posearray_msg = createPoseArrayFromTracks(tracks_msg);
            posearray_pub_->publish(posearray_msg);
        }
        
        // Log periodic status
        if (frame_counter_ % 100 == 0) {
            RCLCPP_INFO(get_logger(), 
                       "Frame %u: %zu clusters, %zu tracks, %zu background bins",
                       frame_counter_, clusters.size(), 
                       tracker_->getActiveTrackCount(),
                       background_model_->getActiveBinCount());
        }
    }

    /**
     * @brief Transform clusters from laser frame to world frame
     * @param clusters Vector of clusters to transform (modified in-place)
     */
    void transformClustersToWorld(std::vector<Cluster>& clusters) {
        for (auto& cluster : clusters) {
            // Transform centroid
            geometry_msgs::msg::PointStamped point_stamped;
            point_stamped.header.frame_id = laser_frame_id_;
            point_stamped.header.stamp = this->now();
            point_stamped.point = cluster.centroid;
            
            try {
                geometry_msgs::msg::PointStamped transformed_point;
                tf_buffer_->transform(point_stamped, transformed_point, world_frame_id_);
                cluster.centroid = transformed_point.point;
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                    "Failed to transform cluster: %s", ex.what());
                continue;
            }
            
            // Transform individual points if needed (currently not used by tracker)
            for (auto& point : cluster.points) {
                geometry_msgs::msg::PointStamped point_stamped_individual;
                point_stamped_individual.header.frame_id = laser_frame_id_;
                point_stamped_individual.header.stamp = this->now();
                point_stamped_individual.point = point;
                
                try {
                    geometry_msgs::msg::PointStamped transformed;
                    tf_buffer_->transform(point_stamped_individual, transformed, world_frame_id_);
                    point = transformed.point;
                } catch (const tf2::TransformException& ex) {
                    // Skip this point if transform fails
                    continue;
                }
            }
        }
    }

    /**
     * @brief Create PoseArray message from tracks for RViz2 debugging
     * @param tracks_msg Tracks message to convert
     * @return PoseArray message with track poses
     */
    geometry_msgs::msg::PoseArray createPoseArrayFromTracks(
        const aatb_msgs::msg::Tracks& tracks_msg) {
        
        geometry_msgs::msg::PoseArray posearray_msg;
        posearray_msg.header = tracks_msg.header;
        
        for (const auto& track : tracks_msg.tracks) {
            posearray_msg.poses.push_back(track.pose);
        }
        
        return posearray_msg;
    }

    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<aatb_msgs::msg::Tracks>::SharedPtr tracks_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr posearray_pub_;
    
    // Transform handling
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped laser_to_world_transform_;
    std::string laser_frame_id_;
    std::string world_frame_id_;
    bool transform_available_ = false;
    
    // Processing components
    std::unique_ptr<BackgroundModel> background_model_;
    std::unique_ptr<ClusterExtractor> cluster_extractor_;
    std::unique_ptr<Tracker> tracker_;
    
    // State
    uint32_t frame_counter_;
    bool enable_background_removal_;
    bool enable_posearray_debug_;
};

} // namespace aatb_crowd_tracker

/**
 * @brief Main function
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<aatb_crowd_tracker::CrowdTrackerNode>();
    
    RCLCPP_INFO(node->get_logger(), "Starting crowd tracker node...");
    
    rclcpp::spin(node);
    
    RCLCPP_INFO(node->get_logger(), "Shutting down crowd tracker node");
    rclcpp::shutdown();
    
    return 0;
}