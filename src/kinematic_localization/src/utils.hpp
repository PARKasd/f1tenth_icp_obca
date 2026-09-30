// MIT License
//
// Copyright (c) 2024 Tiziano Guadagnino, Benedikt Mersch, Ignacio Vizzo, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Adapted from kinematic-icp's kinematic_icp_ros utils (RosUtils.hpp/.cpp,
// TimeStampHandler.hpp/.cpp) for the kinematic_localization package. The TF
// helpers here return std::optional instead of an identity transform so that
// callers can distinguish lookup failures. Also adds the .kissmap binary IO.
#pragma once

#include <tf2/time.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iterator>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <optional>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sophus/se3.hpp>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace kinematic_localization::utils {

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;

// ---------------------------------------------------------------------------
// Sophus <-> ROS conversions
// ---------------------------------------------------------------------------
inline Sophus::SE3d TransformToSophus(const geometry_msgs::msg::TransformStamped &t) {
    return Sophus::SE3d(
        Sophus::SE3d::QuaternionType(t.transform.rotation.w, t.transform.rotation.x,
                                     t.transform.rotation.y, t.transform.rotation.z),
        Sophus::SE3d::Point(t.transform.translation.x, t.transform.translation.y,
                            t.transform.translation.z));
}

inline Sophus::SE3d PoseToSophus(const geometry_msgs::msg::Pose &pose) {
    return Sophus::SE3d(
        Sophus::SE3d::QuaternionType(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                     pose.orientation.z),
        Sophus::SE3d::Point(pose.position.x, pose.position.y, pose.position.z));
}

inline geometry_msgs::msg::Pose SophusToPose(const Sophus::SE3d &T) {
    geometry_msgs::msg::Pose p;
    p.position.x = T.translation().x();
    p.position.y = T.translation().y();
    p.position.z = T.translation().z();
    const Eigen::Quaterniond q(T.so3().unit_quaternion());
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
}

inline geometry_msgs::msg::Transform SophusToTransform(const Sophus::SE3d &T) {
    geometry_msgs::msg::Transform t;
    t.translation.x = T.translation().x();
    t.translation.y = T.translation().y();
    t.translation.z = T.translation().z();
    const Eigen::Quaterniond q(T.so3().unit_quaternion());
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();
    t.rotation.w = q.w();
    return t;
}

// ---------------------------------------------------------------------------
// TF helpers (return std::optional; std::nullopt on lookup failure)
// ---------------------------------------------------------------------------
inline std::optional<Sophus::SE3d> LookupTransform(const std::string &target_frame,
                                                   const std::string &source_frame,
                                                   const tf2_ros::Buffer &tf2_buffer,
                                                   const tf2::TimePoint &time = tf2::TimePointZero) {
    try {
        return TransformToSophus(tf2_buffer.lookupTransform(target_frame, source_frame, time));
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(rclcpp::get_logger("kinematic_localization"), "%s", ex.what());
        return std::nullopt;
    }
}

inline std::optional<Sophus::SE3d> LookupDeltaTransform(const std::string &target_frame,
                                                        const rclcpp::Time &target_time,
                                                        const std::string &source_frame,
                                                        const rclcpp::Time &source_time,
                                                        const std::string &fixed_frame,
                                                        const tf2_ros::Buffer &tf2_buffer) {
    try {
        return TransformToSophus(tf2_buffer.lookupTransform(
            target_frame, target_time, source_frame, source_time, fixed_frame));
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(rclcpp::get_logger("kinematic_localization"), "%s", ex.what());
        return std::nullopt;
    }
}

// ---------------------------------------------------------------------------
// PointCloud2 -> Eigen
// ---------------------------------------------------------------------------
inline std::vector<Eigen::Vector3d> PointCloud2ToEigen(const PointCloud2 &msg) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(msg.height * msg.width);
    sensor_msgs::PointCloud2ConstIterator<float> iter_xyz(msg, "x");
    for (size_t i = 0; i < msg.height * msg.width; ++i, ++iter_xyz) {
        points.emplace_back(static_cast<double>(iter_xyz[0]), static_cast<double>(iter_xyz[1]),
                            static_cast<double>(iter_xyz[2]));
    }
    return points;
}

// ---------------------------------------------------------------------------
// Per-point timestamp extraction (upstream TimeStampHandler, rclcpp::Time based)
// ---------------------------------------------------------------------------
struct TimeStampHandler {
    // Returns (begin_stamp, end_stamp, per-point timestamps normalized to [0, 1])
    std::tuple<rclcpp::Time, rclcpp::Time, std::vector<double>> ProcessTimestamps(
        const PointCloud2 &msg) {
        std::vector<double> timestamps;
        // Find the timestamp field written by laser_geometry (channel_option::Timestamp)
        std::optional<PointField> timestamp_field;
        for (const auto &field : msg.fields) {
            if (field.name == "t" || field.name == "timestamp" || field.name == "time" ||
                field.name == "stamps") {
                timestamp_field = field;
            }
        }
        if (timestamp_field.has_value()) {
            const size_t n_points = msg.height * msg.width;
            timestamps.reserve(n_points);
            if (timestamp_field->datatype == PointField::FLOAT32) {
                sensor_msgs::PointCloud2ConstIterator<float> it(msg, timestamp_field->name);
                for (size_t i = 0; i < n_points; ++i, ++it) timestamps.emplace_back(*it);
            } else if (timestamp_field->datatype == PointField::FLOAT64) {
                sensor_msgs::PointCloud2ConstIterator<double> it(msg, timestamp_field->name);
                for (size_t i = 0; i < n_points; ++i, ++it) timestamps.emplace_back(*it);
            } else if (timestamp_field->datatype == PointField::UINT32) {
                sensor_msgs::PointCloud2ConstIterator<uint32_t> it(msg, timestamp_field->name);
                for (size_t i = 0; i < n_points; ++i, ++it)
                    timestamps.emplace_back(static_cast<double>(*it) * 1e-9);
            } else {
                throw std::runtime_error("timestamp field type not supported");
            }
        } else {
            RCLCPP_WARN_ONCE(rclcpp::get_logger("kinematic_localization"),
                             "No timestamp field in cloud. Disabling scan deskewing");
        }

        const rclcpp::Time msg_stamp(msg.header.stamp);
        const rclcpp::Time begin_stamp = last_processed_stamp_;
        rclcpp::Time end_stamp = msg_stamp;
        if (!timestamps.empty()) {
            const auto &[min_it, max_it] = std::minmax_element(timestamps.cbegin(), timestamps.cend());
            const double min_stamp = *min_it;
            const double max_stamp = *max_it;
            // Check if the message is stamped at the beginning or at the end of the scan
            const bool is_stamped_at_the_beginning =
                std::abs(msg_stamp.seconds() - max_stamp) > 1e-8;
            if (is_stamped_at_the_beginning) {
                end_stamp = msg_stamp + rclcpp::Duration(tf2::durationFromSec(max_stamp - min_stamp));
            }
            // Normalize timestamps to [0, 1]
            const double range = max_stamp - min_stamp;
            if (range > 0.0) {
                std::transform(timestamps.cbegin(), timestamps.cend(), timestamps.begin(),
                               [&](const auto &t) { return (t - min_stamp) / range; });
            }
        }
        last_processed_stamp_ = end_stamp;
        return {begin_stamp, end_stamp, timestamps};
    }

    rclcpp::Time last_processed_stamp_{0, 0, RCL_ROS_TIME};
};

// ---------------------------------------------------------------------------
// .kissmap binary format (custom, defined by this package):
//   8 bytes  magic "KISSMAP1"
//   double   voxel_size
//   double   max_range
//   uint64   point count
//   double x, y, z  x count
// ---------------------------------------------------------------------------
struct KissMap {
    double voxel_size = 0.0;
    double max_range = 0.0;
    std::vector<Eigen::Vector3d> points;
};

inline constexpr char kKissMapMagic[8] = {'K', 'I', 'S', 'S', 'M', 'A', 'P', '1'};

inline bool WriteKissMap(const std::string &path, const KissMap &map) {
    std::ofstream out(path, std::ios::binary);
    if (!out.is_open()) return false;
    out.write(kKissMapMagic, 8);
    out.write(reinterpret_cast<const char *>(&map.voxel_size), sizeof(double));
    out.write(reinterpret_cast<const char *>(&map.max_range), sizeof(double));
    const uint64_t count = map.points.size();
    out.write(reinterpret_cast<const char *>(&count), sizeof(uint64_t));
    for (const auto &p : map.points) {
        out.write(reinterpret_cast<const char *>(p.data()), 3 * sizeof(double));
    }
    return out.good();
}

inline std::optional<KissMap> ReadKissMap(const std::string &path) {
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) return std::nullopt;
    char magic[8];
    in.read(magic, 8);
    if (!in.good() || !std::equal(std::begin(magic), std::end(magic), std::begin(kKissMapMagic))) {
        return std::nullopt;
    }
    KissMap map;
    uint64_t count = 0;
    in.read(reinterpret_cast<char *>(&map.voxel_size), sizeof(double));
    in.read(reinterpret_cast<char *>(&map.max_range), sizeof(double));
    in.read(reinterpret_cast<char *>(&count), sizeof(uint64_t));
    if (!in.good()) return std::nullopt;
    map.points.resize(count);
    for (auto &p : map.points) {
        in.read(reinterpret_cast<char *>(p.data()), 3 * sizeof(double));
    }
    if (!in.good()) return std::nullopt;
    return map;
}

// ---------------------------------------------------------------------------
// Occupancy grid rasterization (built-in /map server)
// ---------------------------------------------------------------------------
// Rasterize map-frame points into an occupancy grid: each point paints a disk
// of radius dilation_m; painted cells are 100 (occupied), everything else is
// -1 (unknown — points alone say nothing about free space). The grid covers
// the point bounding box plus margin_m on each side. resolution has the same
// meaning as the map_server yaml 'resolution' (m/cell), so a grid built with
// the source map's resolution matches scripts/pgm_to_kissmap.py geometry.
inline nav_msgs::msg::OccupancyGrid RasterizeOccupancyGrid(
    const std::vector<Eigen::Vector3d> &points, double resolution, double dilation_m,
    double margin_m, const std::string &frame_id, const rclcpp::Time &stamp) {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = frame_id;
    grid.header.stamp = stamp;
    if (points.empty() || resolution <= 0.0) return grid;

    double min_x = points.front().x(), max_x = min_x;
    double min_y = points.front().y(), max_y = min_y;
    for (const auto &p : points) {
        min_x = std::min(min_x, p.x());
        max_x = std::max(max_x, p.x());
        min_y = std::min(min_y, p.y());
        max_y = std::max(max_y, p.y());
    }
    const double origin_x = min_x - margin_m;
    const double origin_y = min_y - margin_m;
    const uint32_t width =
        static_cast<uint32_t>(std::ceil((max_x - min_x + 2.0 * margin_m) / resolution)) + 1;
    const uint32_t height =
        static_cast<uint32_t>(std::ceil((max_y - min_y + 2.0 * margin_m) / resolution)) + 1;

    grid.info.resolution = static_cast<float>(resolution);
    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position.x = origin_x;
    grid.info.origin.position.y = origin_y;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(static_cast<size_t>(width) * height, -1);

    const int r = static_cast<int>(std::ceil(dilation_m / resolution));
    const double r2 = dilation_m * dilation_m;
    for (const auto &p : points) {
        const int cx = static_cast<int>(std::floor((p.x() - origin_x) / resolution));
        const int cy = static_cast<int>(std::floor((p.y() - origin_y) / resolution));
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                const double ddx = dx * resolution, ddy = dy * resolution;
                if (ddx * ddx + ddy * ddy > r2) continue;
                const int x = cx + dx, y = cy + dy;
                if (x < 0 || y < 0 || x >= static_cast<int>(width) ||
                    y >= static_cast<int>(height))
                    continue;
                grid.data[static_cast<size_t>(y) * width + x] = 100;
            }
        }
    }
    return grid;
}

// ---------------------------------------------------------------------------
// Chassis roll/pitch compensation for a planar LiDAR
// ---------------------------------------------------------------------------
// A cornering chassis tilts the 2D scan plane with the sensor. Estimate the
// steady-state attitude from wheel odometry instead of an accelerometer, whose
// lateral axis cannot separate gravity from centripetal acceleration here:
//
//   roll = roll_gradient * a_lat,  a_lat = v * yaw_rate
//
// The attitude is expressed in base_link axes, so conjugate it through the
// LiDAR extrinsic before rotating LiDAR-frame points. The frozen map is planar;
// after measuring optional out-of-plane rejection in base axes, project the
// compensated points back to z=0.
struct TiltParams {
    double roll_gradient_rad_per_mps2 = 0.0;
    double pitch_gradient_rad_per_mps2 = 0.0;
    double max_angle_rad = 0.26;
    double max_point_height_m = 0.0;
};

struct TiltResult {
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    size_t dropped = 0;
    // Populated only when point-height rejection is enabled, so callers can
    // keep per-point deskew timestamps aligned with a filtered cloud.
    std::vector<size_t> kept_indices;
};

// a_lat [m/s^2]: positive in a left turn. a_lon [m/s^2]: positive accelerating.
inline std::vector<Eigen::Vector3d> LevelScan(const std::vector<Eigen::Vector3d> &points,
                                              const Sophus::SE3d &lidar_to_base, double a_lat,
                                              double a_lon, const TiltParams &cfg,
                                              TiltResult *out) {
    TiltResult result;
    // Positive roll raises the left side; positive pitch lowers the nose.
    result.roll_rad = std::clamp(cfg.roll_gradient_rad_per_mps2 * a_lat,
                                 -cfg.max_angle_rad, cfg.max_angle_rad);
    result.pitch_rad = std::clamp(cfg.pitch_gradient_rad_per_mps2 * (-a_lon),
                                  -cfg.max_angle_rad, cfg.max_angle_rad);
    if (out) *out = result;
    if (std::abs(result.roll_rad) < 1e-4 && std::abs(result.pitch_rad) < 1e-4) {
        return points;
    }

    const Eigen::Matrix3d R_base =
        (Eigen::AngleAxisd(result.pitch_rad, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(result.roll_rad, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    const Eigen::Matrix3d R_lidar_to_base = lidar_to_base.so3().matrix();
    const Eigen::Matrix3d R_lidar =
        R_lidar_to_base.transpose() * R_base * R_lidar_to_base;

    std::vector<Eigen::Vector3d> leveled;
    leveled.reserve(points.size());
    const bool reject_out_of_plane = cfg.max_point_height_m > 1e-6;
    for (size_t index = 0; index < points.size(); ++index) {
        const auto &point = points[index];
        if (reject_out_of_plane) {
            const Eigen::Vector3d point_base = R_lidar_to_base * point;
            if (std::abs((R_base * point_base).z() - point_base.z()) >
                cfg.max_point_height_m) {
                ++result.dropped;
                continue;
            }
            result.kept_indices.push_back(index);
        }
        Eigen::Vector3d corrected = R_lidar * point;
        corrected.z() = 0.0;
        leveled.push_back(corrected);
    }
    if (out) *out = result;
    // A bad rejection threshold must not starve ICP of its entire source cloud.
    if (leveled.size() < 10) return points;
    return leveled;
}

}  // namespace kinematic_localization::utils
