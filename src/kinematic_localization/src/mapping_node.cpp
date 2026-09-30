// Offline mapping node (2026_IFAC): builds a .kissmap point map from a rosbag.
//
// Reads the bag sequentially (message-count based, no time windows — the
// sqlite3 send_timestamp==0 bug of upstream's BufferableBag cannot occur),
// loads all /tf + /tf_static into a TF buffer in a first pass, then runs the
// kinematic-icp pipeline (freeze_local_map = false) over every scan and dumps
// the accumulated, downsampled point map to a .kissmap file.
#include <tf2_ros/buffer.h>

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <deque>
#include <laser_geometry/laser_geometry.hpp>
#include <kiss_icp/core/Preprocessing.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sophus/se3.hpp>
#include <stdexcept>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>
#include <vector>

#include "kinematic_icp/pipeline/KinematicICP.hpp"
#include "utils.hpp"

namespace kinematic_localization {

namespace {
constexpr int kExpectedConfigSchemaVersion = 20260824;
}  // namespace

class MappingNode : public rclcpp::Node {
public:
    MappingNode() : Node("kinematic_mapping") {
        const int config_schema_version =
            declare_parameter<int>("config_schema_version", 0);
        if (config_schema_version != kExpectedConfigSchemaVersion) {
            const std::string message =
                "config_schema_version mismatch: expected " +
                std::to_string(kExpectedConfigSchemaVersion) + ", got " +
                std::to_string(config_schema_version) +
                ". Launch with the installed config/mapping.yaml.";
            RCLCPP_FATAL(get_logger(), "%s", message.c_str());
            throw std::runtime_error(message);
        }

        bag_path_ = declare_parameter<std::string>("bag_path", "");
        lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/scan");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
        output_path_ = declare_parameter<std::string>("output_path", "map.kissmap");
        tf_topic_ = declare_parameter<std::string>("tf_topic", "/tf");
        tf_static_topic_ = declare_parameter<std::string>("tf_static_topic", "/tf_static");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

        kinematic_icp::pipeline::Config config;
        config.max_range = declare_parameter<double>("max_range", 30.0);
        config.min_range = declare_parameter<double>("min_range", 0.1);
        config.voxel_size = declare_parameter<double>("voxel_size", 1.0);
        config.max_points_per_voxel =
            declare_parameter<int>("max_points_per_voxel", config.max_points_per_voxel);
        config.use_adaptive_threshold =
            declare_parameter<bool>("use_adaptive_threshold", config.use_adaptive_threshold);
        config.fixed_threshold = declare_parameter<double>("fixed_threshold", 1.0);
        config.max_num_iterations = declare_parameter<int>("max_num_iterations", 30);
        config.convergence_criterion =
            declare_parameter<double>("convergence_criterion", config.convergence_criterion);
        config.max_num_threads =
            declare_parameter<int>("max_num_threads", config.max_num_threads);
        config.use_adaptive_odometry_regularization = declare_parameter<bool>(
            "use_adaptive_odometry_regularization", config.use_adaptive_odometry_regularization);
        config.fixed_regularization = declare_parameter<double>("fixed_regularization", 0.0);
        config.deskew = declare_parameter<bool>("deskew", true);
        tilt_compensation_enable_ =
            declare_parameter<bool>("tilt_compensation_enable", false);
        tilt_cfg_.roll_gradient_rad_per_mps2 =
            declare_parameter<double>("roll_gradient_rad_per_mps2", 0.0270);
        tilt_cfg_.pitch_gradient_rad_per_mps2 =
            declare_parameter<double>("pitch_gradient_rad_per_mps2", 0.0);
        tilt_cfg_.max_angle_rad =
            std::max(0.0, declare_parameter<double>("tilt_max_angle_rad", 0.26));
        tilt_cfg_.max_point_height_m =
            std::max(0.0, declare_parameter<double>("tilt_max_point_height_m", 0.0));
        config.freeze_local_map = false;  // mapping mode: scans build the map
        voxel_size_ = config.voxel_size;
        max_range_ = config.max_range;

        RCLCPP_INFO(
            get_logger(),
            "E2 parameter summary | schema=%d max_range=%.1f min_range=%.1f "
            "voxel_size=%.3f max_points_per_voxel=%d iterations=%d deskew=%s "
            "tilt=%s roll_gradient=%.4f",
            config_schema_version, config.max_range, config.min_range, config.voxel_size,
            config.max_points_per_voxel, config.max_num_iterations,
            config.deskew ? "true" : "false",
            tilt_compensation_enable_ ? "true" : "false",
            tilt_cfg_.roll_gradient_rad_per_mps2);

        icp_ = std::make_unique<kinematic_icp::pipeline::KinematicICP>(config);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    }

    int Run() {
        if (bag_path_.empty()) {
            RCLCPP_ERROR(get_logger(), "bag_path parameter is required");
            return 1;
        }

        // Pass 1: read the whole bag. TF goes into the buffer, scans are stored.
        // Processing is message-count based; bag send/receive timestamps are never used.
        rosbag2_cpp::Reader reader;
        reader.open(bag_path_);
        rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serializer;
        rclcpp::Serialization<sensor_msgs::msg::LaserScan> scan_serializer;
        rclcpp::Serialization<nav_msgs::msg::Odometry> odom_serializer;
        std::vector<sensor_msgs::msg::LaserScan> scans;
        std::deque<std::pair<rclcpp::Time, Sophus::SE3d>> odom_history;
        while (reader.has_next()) {
            const auto msg = reader.read_next();
            rclcpp::SerializedMessage serialized(*msg->serialized_data);
            if (msg->topic_name == tf_topic_ || msg->topic_name == tf_static_topic_) {
                tf2_msgs::msg::TFMessage tf_msg;
                tf_serializer.deserialize_message(&serialized, &tf_msg);
                for (const auto &t : tf_msg.transforms) {
                    tf_buffer_->setTransform(t, "bag", msg->topic_name == tf_static_topic_);
                }
            } else if (msg->topic_name == lidar_topic_) {
                sensor_msgs::msg::LaserScan scan;
                scan_serializer.deserialize_message(&serialized, &scan);
                scans.push_back(std::move(scan));
            } else if (msg->topic_name == odom_topic_) {
                nav_msgs::msg::Odometry odom;
                odom_serializer.deserialize_message(&serialized, &odom);
                odom_history.emplace_back(rclcpp::Time(odom.header.stamp),
                                          utils::PoseToSophus(odom.pose.pose));
            }
        }
        RCLCPP_INFO(get_logger(), "Loaded %zu scans, %zu odom msgs from %s", scans.size(),
                    odom_history.size(), bag_path_.c_str());
        if (scans.empty() || odom_history.empty()) return 1;

        // Nearest odom pose within 100 ms of a stamp
        const auto odom_at = [&](const rclcpp::Time &stamp) -> std::optional<Sophus::SE3d> {
            auto best = odom_history.begin();
            double best_dt = std::abs((best->first - stamp).seconds());
            for (auto it = odom_history.begin() + 1; it != odom_history.end(); ++it) {
                const double d = std::abs((it->first - stamp).seconds());
                if (d < best_dt) {
                    best = it;
                    best_dt = d;
                } else if (it->first > stamp) {
                    break;
                }
            }
            if (best_dt > 0.1) return std::nullopt;
            return best->second;
        };

        // Pass 2: run the pipeline over every scan
        std::vector<Eigen::Vector3d> accumulated;
        std::optional<Sophus::SE3d> lidar_to_base;
        bool pose_initialized = false;
        size_t processed = 0;
        double last_speed = 0.0;
        bool has_last_speed = false;
        utils::TimeStampHandler timestamps_handler;
        laser_geometry::LaserProjection laser_projector;

        for (const auto &scan : scans) {
            if (!lidar_to_base.has_value()) {
                const auto extrinsic =
                    utils::LookupTransform(base_frame_, scan.header.frame_id, *tf_buffer_);
                if (!extrinsic.has_value()) continue;
                lidar_to_base = extrinsic;
            }
            sensor_msgs::msg::PointCloud2 cloud;
            laser_projector.projectLaser(scan, cloud, -1.0,
                                         laser_geometry::channel_option::Timestamp);
            const auto points = utils::PointCloud2ToEigen(cloud);
            // KISS deskew references points to the scan end; the odometry
            // window and the accumulated pose use the handler's [begin, end].
            const auto &[begin_stamp, end_stamp, timestamps] =
                timestamps_handler.ProcessTimestamps(cloud);

            if (!pose_initialized) {
                // Start the map at the current wheel odometry pose (map frame == odom frame)
                const auto T_odom_base = odom_at(end_stamp);
                if (!T_odom_base.has_value()) continue;
                icp_->SetPose(*T_odom_base);
                pose_initialized = true;
                continue;
            }

            const auto T_begin = odom_at(begin_stamp);
            const auto T_end = odom_at(end_stamp);
            if (!T_begin.has_value() || !T_end.has_value()) continue;
            const Sophus::SE3d delta = T_begin->inverse() * (*T_end);
            if (delta.log().norm() <= 1e-3) continue;

            std::vector<Eigen::Vector3d> registration_points = points;
            std::vector<double> registration_timestamps = timestamps;
            const double dt = (end_stamp - begin_stamp).seconds();
            if (tilt_compensation_enable_ && dt > 1e-6) {
                const double speed = delta.translation().norm() / dt;
                const double lateral_acceleration = speed * (delta.so3().log().z() / dt);
                const double longitudinal_acceleration =
                    has_last_speed ? (speed - last_speed) / dt : 0.0;
                last_speed = speed;
                has_last_speed = true;
                utils::TiltResult tilt_result;
                registration_points = utils::LevelScan(
                    points, *lidar_to_base, lateral_acceleration, longitudinal_acceleration,
                    tilt_cfg_, &tilt_result);
                if (registration_points.size() != points.size()) {
                    registration_timestamps.clear();
                    if (timestamps.size() == points.size() &&
                        tilt_result.kept_indices.size() == registration_points.size()) {
                        registration_timestamps.reserve(tilt_result.kept_indices.size());
                        for (const size_t index : tilt_result.kept_indices) {
                            registration_timestamps.push_back(timestamps[index]);
                        }
                    }
                }
            }

            const auto &result =
                icp_->RegisterFrame(registration_points, registration_timestamps,
                                    *lidar_to_base, delta);
            // Accumulate the downsampled registration points in the map frame
            const Sophus::SE3d &pose = icp_->pose();
            for (const auto &p : std::get<1>(result)) accumulated.push_back(pose * p);
            if (++processed % 200 == 0) {
                RCLCPP_INFO(get_logger(), "Processed %zu / %zu scans", processed, scans.size());
            }
        }
        RCLCPP_INFO(get_logger(), "Registration done: %zu scans, %zu raw map points", processed,
                    accumulated.size());

        // Final voxel downsample before saving
        auto downsampled = kiss_icp::VoxelDownsample(accumulated, voxel_size_ * 0.5);
        utils::KissMap map;
        map.voxel_size = voxel_size_;
        map.max_range = max_range_;
        map.points = std::move(downsampled);
        if (!utils::WriteKissMap(output_path_, map)) {
            RCLCPP_ERROR(get_logger(), "Failed to write %s", output_path_.c_str());
            return 1;
        }
        RCLCPP_INFO(get_logger(), "Wrote %zu points to %s", map.points.size(),
                    output_path_.c_str());
        return 0;
    }

private:
    std::string bag_path_, lidar_topic_, odom_topic_, output_path_, tf_topic_, tf_static_topic_;
    std::string base_frame_;
    double voxel_size_ = 1.0, max_range_ = 30.0;
    bool tilt_compensation_enable_ = false;
    utils::TiltParams tilt_cfg_;
    std::unique_ptr<kinematic_icp::pipeline::KinematicICP> icp_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace kinematic_localization

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<kinematic_localization::MappingNode>();
    const int rc = node->Run();
    rclcpp::shutdown();
    return rc;
}
