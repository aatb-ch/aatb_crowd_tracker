#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <aatb_msgs/msg/tracks.hpp>

#include "aatb_crowd_tracker/background_model.hpp"
#include "aatb_crowd_tracker/cluster_extractor.hpp"
#include "aatb_crowd_tracker/tracker.hpp"
#include "aatb_crowd_tracker/heatmap_model.hpp"

#include <cmath>

namespace aatb_crowd_tracker {

class CrowdTrackerNode : public rclcpp::Node {
public:
    CrowdTrackerNode() : Node("crowd_tracker_node"), frame_counter_(0) {
        initializeParameters();
        initializeTransforms();
        initializeComponents();
        initializePubSub();
        RCLCPP_INFO(get_logger(), "CrowdTracker node initialized");
    }

private:
    // -------------------------------------------------------------------------
    // Initialization
    // -------------------------------------------------------------------------

    void initializeParameters() {
        declare_parameter("background_model.learning_rate", 0.01);
        declare_parameter("background_model.occupancy_threshold", 0.7);
        declare_parameter("background_model.decay_frames", 100);
        declare_parameter("background_model.decay_rate", 0.95);
        declare_parameter("background_model.enable", true);

        declare_parameter("cluster_extractor.min_cluster_size", 3);
        declare_parameter("cluster_extractor.max_cluster_size", 100);
        declare_parameter("cluster_extractor.base_distance_threshold", 0.3);
        declare_parameter("cluster_extractor.distance_scaling_factor", 0.1);

        declare_parameter("tracker.association_threshold", 1.0);
        declare_parameter("tracker.max_consecutive_misses", 10);
        declare_parameter("tracker.min_track_confidence", 0.5);
        declare_parameter("tracker.movement_threshold", 0.1);

        declare_parameter("geofence.enable", false);
        declare_parameter("geofence.min_x", -10.0);
        declare_parameter("geofence.max_x", 10.0);
        declare_parameter("geofence.min_y", -10.0);
        declare_parameter("geofence.max_y", 10.0);

        declare_parameter("heatmap.enable", true);
        declare_parameter("heatmap.resolution", 0.2);
        declare_parameter("heatmap.max_range", 15.0);
        declare_parameter("heatmap.slow_learning_rate", 0.002);
        declare_parameter("heatmap.fast_learning_rate", 0.05);
        declare_parameter("heatmap.slow_decay_rate", 0.001);
        declare_parameter("heatmap.fast_decay_rate", 0.02);
        declare_parameter("heatmap.schmitt_high", 0.10);
        declare_parameter("heatmap.schmitt_low", 0.02);
        declare_parameter("heatmap.derivative_gate", 0.005);
        declare_parameter("heatmap.min_blob_cells", 2);
        declare_parameter("heatmap.max_blob_cells", 500);
        declare_parameter("heatmap.heatmap_topic", "/heatmap");
        declare_parameter("heatmap.active_topic", "/heatmap_active");
        declare_parameter("heatmap.tracks_topic", "/heatmap_tracks");
        declare_parameter("heatmap.tracks_posearray_topic", "/heatmap_tracks_posearray");
        declare_parameter("heatmap.tracker.association_threshold", 1.5);
        declare_parameter("heatmap.tracker.max_consecutive_misses", 5);
        declare_parameter("heatmap.tracker.min_track_confidence", 0.5);
        declare_parameter("heatmap.scan_timeout", 1.0);

        declare_parameter("background_model.reset_interval_minutes", 30);

        declare_parameter("scan_topic", "/scan");
        declare_parameter("tracks_topic", "/tracks");
        declare_parameter("posearray_topic", "/tracks_posearray");
        declare_parameter("enable_posearray_debug", true);
        declare_parameter("output_frame_id", "world");
        declare_parameter("publish_rate", 10.0);
    }

    void initializeTransforms() {
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        output_frame_id_ = get_parameter("output_frame_id").as_string();
        RCLCPP_INFO(get_logger(), "Output frame: %s", output_frame_id_.c_str());
    }

    void initializeComponents() {
        background_model_ = std::make_unique<BackgroundModel>(
            get_parameter("background_model.learning_rate").as_double(),
            get_parameter("background_model.occupancy_threshold").as_double(),
            get_parameter("background_model.decay_frames").as_int(),
            get_parameter("background_model.decay_rate").as_double());

        cluster_extractor_ = std::make_unique<ClusterExtractor>(
            get_parameter("cluster_extractor.min_cluster_size").as_int(),
            get_parameter("cluster_extractor.max_cluster_size").as_int(),
            get_parameter("cluster_extractor.base_distance_threshold").as_double(),
            get_parameter("cluster_extractor.distance_scaling_factor").as_double());

        tracker_ = std::make_unique<Tracker>(
            this,
            get_parameter("tracker.association_threshold").as_double(),
            get_parameter("tracker.max_consecutive_misses").as_int(),
            get_parameter("tracker.min_track_confidence").as_double(),
            get_parameter("tracker.movement_threshold").as_double(),
            /*use_velocity_prediction=*/true);

        enable_background_removal_ = get_parameter("background_model.enable").as_bool();
        enable_posearray_debug_    = get_parameter("enable_posearray_debug").as_bool();
        enable_geofence_           = get_parameter("geofence.enable").as_bool();
        geofence_min_x_            = get_parameter("geofence.min_x").as_double();
        geofence_max_x_            = get_parameter("geofence.max_x").as_double();
        geofence_min_y_            = get_parameter("geofence.min_y").as_double();
        geofence_max_y_            = get_parameter("geofence.max_y").as_double();

        if (enable_geofence_) {
            RCLCPP_INFO(get_logger(), "Geofence enabled: X[%.2f, %.2f] Y[%.2f, %.2f]",
                geofence_min_x_, geofence_max_x_, geofence_min_y_, geofence_max_y_);
        }

        enable_heatmap_ = get_parameter("heatmap.enable").as_bool();
        if (enable_heatmap_) {
            initializeHeatmap();
        }
    }

    void initializeHeatmap() {
        HeatmapModel::Config cfg;
        cfg.resolution         = static_cast<float>(get_parameter("heatmap.resolution").as_double());
        cfg.slow_learning_rate = static_cast<float>(get_parameter("heatmap.slow_learning_rate").as_double());
        cfg.fast_learning_rate = static_cast<float>(get_parameter("heatmap.fast_learning_rate").as_double());
        cfg.slow_decay_rate    = static_cast<float>(get_parameter("heatmap.slow_decay_rate").as_double());
        cfg.fast_decay_rate    = static_cast<float>(get_parameter("heatmap.fast_decay_rate").as_double());
        cfg.schmitt_high       = static_cast<float>(get_parameter("heatmap.schmitt_high").as_double());
        cfg.schmitt_low        = static_cast<float>(get_parameter("heatmap.schmitt_low").as_double());
        cfg.derivative_gate    = static_cast<float>(get_parameter("heatmap.derivative_gate").as_double());
        cfg.min_blob_cells     = get_parameter("heatmap.min_blob_cells").as_int();
        cfg.max_blob_cells     = get_parameter("heatmap.max_blob_cells").as_int();

        if (enable_geofence_) {
            cfg.origin_x = static_cast<float>(geofence_min_x_);
            cfg.origin_y = static_cast<float>(geofence_min_y_);
            cfg.width    = static_cast<int>(std::ceil((geofence_max_x_ - geofence_min_x_) / cfg.resolution));
            cfg.height   = static_cast<int>(std::ceil((geofence_max_y_ - geofence_min_y_) / cfg.resolution));
        } else {
            const float max_range = static_cast<float>(get_parameter("heatmap.max_range").as_double());
            cfg.origin_x = -max_range;
            cfg.origin_y = -max_range;
            cfg.width    = static_cast<int>(std::ceil(2.0f * max_range / cfg.resolution));
            cfg.height   = static_cast<int>(std::ceil(2.0f * max_range / cfg.resolution));
        }

        const int cell_count = cfg.width * cfg.height;
        RCLCPP_INFO(get_logger(),
            "Heatmap: %dx%d cells (%.1fx%.1f m at %.2f m/cell) — %d cells total",
            cfg.width, cfg.height,
            cfg.width * cfg.resolution, cfg.height * cfg.resolution,
            cfg.resolution, cell_count);

        if (cell_count > 500000) {
            RCLCPP_WARN(get_logger(),
                "Heatmap has %d cells — consider increasing resolution or reducing range "
                "to stay under 500K cells for best performance on embedded hardware", cell_count);
        }

        heatmap_model_ = std::make_unique<HeatmapModel>(cfg);

        // Heatmap blob tracker: no velocity prediction to avoid ghost extrapolation
        heatmap_tracker_ = std::make_unique<Tracker>(
            this,
            get_parameter("heatmap.tracker.association_threshold").as_double(),
            get_parameter("heatmap.tracker.max_consecutive_misses").as_int(),
            get_parameter("heatmap.tracker.min_track_confidence").as_double(),
            /*movement_threshold=*/0.0,
            /*use_velocity_prediction=*/false);

        heatmap_scan_timeout_ = get_parameter("heatmap.scan_timeout").as_double();
        last_scan_time_       = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    void initializePubSub() {
        const auto scan_topic      = get_parameter("scan_topic").as_string();
        const auto tracks_topic    = get_parameter("tracks_topic").as_string();
        const auto posearray_topic = get_parameter("posearray_topic").as_string();

        laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                processLaserScan(msg);
            });

        tracks_pub_ = create_publisher<aatb_msgs::msg::Tracks>(tracks_topic, 10);

        if (enable_posearray_debug_) {
            posearray_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(posearray_topic, 10);
        }

        if (enable_heatmap_) {
            heatmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
                get_parameter("heatmap.heatmap_topic").as_string(), 1);
            active_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
                get_parameter("heatmap.active_topic").as_string(), 1);
            heatmap_tracks_pub_ = create_publisher<aatb_msgs::msg::Tracks>(
                get_parameter("heatmap.tracks_topic").as_string(), 10);
            heatmap_tracks_posearray_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
                get_parameter("heatmap.tracks_posearray_topic").as_string(), 10);

            // 10 Hz fallback timer for decay when no scan is received
            fallback_timer_ = create_wall_timer(
                std::chrono::milliseconds(100),
                [this]() { heatmapFallbackTick(); });
        }

        // Periodic background model reset to prevent long-term sensitivity loss
        const int reset_minutes = get_parameter("background_model.reset_interval_minutes").as_int();
        if (reset_minutes > 0 && enable_background_removal_) {
            bg_reset_timer_ = create_wall_timer(
                std::chrono::minutes(reset_minutes),
                [this, reset_minutes]() {
                    background_model_->clear();
                    RCLCPP_INFO(get_logger(),
                        "Background model reset (every %d min) — re-learning from scratch",
                        reset_minutes);
                });
            RCLCPP_INFO(get_logger(), "Background model will reset every %d minutes", reset_minutes);
        }

        RCLCPP_INFO(get_logger(), "Subscribed to %s, publishing tracks to %s",
            scan_topic.c_str(), tracks_topic.c_str());
    }

    // -------------------------------------------------------------------------
    // Scan processing
    // -------------------------------------------------------------------------

    void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        frame_counter_++;

        if (enable_background_removal_) {
            background_model_->update(*msg, frame_counter_);
        }

        std::vector<Cluster> clusters;
        if (enable_background_removal_) {
            clusters = cluster_extractor_->extractClusters(*msg, *background_model_);
        } else {
            BackgroundModel dummy(0.0, 2.0, 1000, 1.0);
            clusters = cluster_extractor_->extractClusters(*msg, dummy);
        }

        transformClusters(clusters, msg->header.frame_id, msg->header.stamp);

        auto tracks_msg = tracker_->updateTracks(clusters, msg->header.stamp);
        tracks_msg.header.frame_id = output_frame_id_;

        if (enable_geofence_) {
            applyGeofenceFilter(tracks_msg);
        }

        tracks_pub_->publish(tracks_msg);

        if (enable_posearray_debug_ && posearray_pub_) {
            posearray_pub_->publish(createPoseArrayFromTracks(tracks_msg));
        }

        if (enable_heatmap_ && heatmap_model_) {
            const auto scan_points = scanToWorldPoints(*msg);
            heatmap_model_->updateFromPoints(scan_points);
            last_scan_time_ = msg->header.stamp;
            publishHeatmapsAndTracks(msg->header.stamp);
        }

        if (frame_counter_ % 100 == 0) {
            // Periodic cleanup: prune stale bins from the background map
            if (enable_background_removal_ && frame_counter_ % 1000 == 0) {
                background_model_->cleanup();
            }

            RCLCPP_INFO(get_logger(),
                "Frame %u: %zu clusters, %zu tracks, %zu bg bins",
                frame_counter_, clusters.size(),
                tracker_->getActiveTrackCount(),
                background_model_->getActiveBinCount());
        }
    }

    // Convert laser scan to XY points in the output frame (single TF lookup per scan)
    std::vector<geometry_msgs::msg::Point> scanToWorldPoints(
        const sensor_msgs::msg::LaserScan& scan)
    {
        std::vector<geometry_msgs::msg::Point> points;
        points.reserve(scan.ranges.size());

        geometry_msgs::msg::TransformStamped transform;
        bool has_transform = false;
        if (scan.header.frame_id != output_frame_id_) {
            try {
                transform = tf_buffer_->lookupTransform(
                    output_frame_id_, scan.header.frame_id,
                    scan.header.stamp, tf2::durationFromSec(0.1));
                has_transform = true;
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Heatmap TF %s->%s: %s",
                    scan.header.frame_id.c_str(), output_frame_id_.c_str(), ex.what());
                return points;
            }
        }

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            const float r = scan.ranges[i];
            if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;

            const float angle = scan.angle_min + static_cast<float>(i) * scan.angle_increment;
            geometry_msgs::msg::PointStamped pt_in;
            pt_in.header  = scan.header;
            pt_in.point.x = r * std::cos(angle);
            pt_in.point.y = r * std::sin(angle);
            pt_in.point.z = 0.0;

            if (has_transform) {
                geometry_msgs::msg::PointStamped pt_out;
                try {
                    tf2::doTransform(pt_in, pt_out, transform);
                    points.push_back(pt_out.point);
                } catch (const tf2::TransformException&) {
                    continue;
                }
            } else {
                points.push_back(pt_in.point);
            }
        }
        return points;
    }

    void publishHeatmapsAndTracks(const builtin_interfaces::msg::Time& stamp) {
        std_msgs::msg::Header h;
        h.stamp    = stamp;
        h.frame_id = output_frame_id_;

        heatmap_pub_->publish(heatmap_model_->buildHeatmapGrid(h));
        active_pub_->publish(heatmap_model_->buildActiveGrid(h));

        auto blobs = heatmap_model_->extractBlobs();
        auto ht    = heatmap_tracker_->updateTracks(blobs, stamp);
        ht.header.frame_id = output_frame_id_;
        heatmap_tracks_pub_->publish(ht);
        heatmap_tracks_posearray_pub_->publish(createPoseArrayFromTracks(ht));
    }

    // Fallback: runs at 10 Hz; applies decay-only update after scan timeout
    void heatmapFallbackTick() {
        if (!enable_heatmap_ || !heatmap_model_) return;
        if (last_scan_time_.nanoseconds() == 0) return;  // no scan received yet

        const rclcpp::Time now = get_clock()->now();
        if ((now - last_scan_time_).seconds() > heatmap_scan_timeout_) {
            heatmap_model_->decayUpdate();
            publishHeatmapsAndTracks(now);
        }
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    void transformClusters(std::vector<Cluster>& clusters,
                           const std::string& source_frame_id,
                           const builtin_interfaces::msg::Time& stamp) {
        if (source_frame_id == output_frame_id_) return;

        for (auto& cluster : clusters) {
            geometry_msgs::msg::PointStamped ps;
            ps.header.frame_id = source_frame_id;
            ps.header.stamp    = stamp;
            ps.point           = cluster.centroid;

            try {
                geometry_msgs::msg::PointStamped out;
                tf_buffer_->transform(ps, out, output_frame_id_, tf2::durationFromSec(0.1));
                cluster.centroid = out.point;
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Failed to transform cluster %s->%s: %s",
                    source_frame_id.c_str(), output_frame_id_.c_str(), ex.what());
                continue;
            }

            for (auto& point : cluster.points) {
                geometry_msgs::msg::PointStamped psi;
                psi.header.frame_id = source_frame_id;
                psi.header.stamp    = stamp;
                psi.point           = point;
                try {
                    geometry_msgs::msg::PointStamped out;
                    tf_buffer_->transform(psi, out, output_frame_id_, tf2::durationFromSec(0.1));
                    point = out.point;
                } catch (const tf2::TransformException&) {
                    continue;
                }
            }
        }
    }

    void applyGeofenceFilter(aatb_msgs::msg::Tracks& tracks_msg) {
        auto it = tracks_msg.tracks.begin();
        while (it != tracks_msg.tracks.end()) {
            const auto& pos = it->pose.position;
            if (pos.x < geofence_min_x_ || pos.x > geofence_max_x_ ||
                pos.y < geofence_min_y_ || pos.y > geofence_max_y_) {
                it = tracks_msg.tracks.erase(it);
            } else {
                ++it;
            }
        }
    }

    geometry_msgs::msg::PoseArray createPoseArrayFromTracks(
        const aatb_msgs::msg::Tracks& tracks_msg)
    {
        geometry_msgs::msg::PoseArray pa;
        pa.header = tracks_msg.header;
        for (const auto& t : tracks_msg.tracks) {
            pa.poses.push_back(t.pose);
        }
        return pa;
    }

    // -------------------------------------------------------------------------
    // Members
    // -------------------------------------------------------------------------

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<aatb_msgs::msg::Tracks>::SharedPtr         tracks_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr  posearray_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr   heatmap_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr   active_pub_;
    rclcpp::Publisher<aatb_msgs::msg::Tracks>::SharedPtr         heatmap_tracks_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr  heatmap_tracks_posearray_pub_;
    rclcpp::TimerBase::SharedPtr                                 fallback_timer_;
    rclcpp::TimerBase::SharedPtr                                 bg_reset_timer_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string output_frame_id_;

    std::unique_ptr<BackgroundModel>  background_model_;
    std::unique_ptr<ClusterExtractor> cluster_extractor_;
    std::unique_ptr<Tracker>          tracker_;
    std::unique_ptr<HeatmapModel>     heatmap_model_;
    std::unique_ptr<Tracker>          heatmap_tracker_;

    uint32_t     frame_counter_;
    rclcpp::Time last_scan_time_;
    double       heatmap_scan_timeout_ = 1.0;

    bool enable_background_removal_;
    bool enable_posearray_debug_;
    bool enable_heatmap_ = false;
    bool enable_geofence_;
    double geofence_min_x_, geofence_max_x_;
    double geofence_min_y_, geofence_max_y_;
};

} // namespace aatb_crowd_tracker

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<aatb_crowd_tracker::CrowdTrackerNode>();
    RCLCPP_INFO(node->get_logger(), "Starting crowd tracker node...");
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "Shutting down crowd tracker node");
    rclcpp::shutdown();
    return 0;
}
