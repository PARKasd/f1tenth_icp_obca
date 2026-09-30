// MIT License

// Copyright (c) 2024 Tiziano Guadagnino, Benedikt Mersch, Ignacio Vizzo, Cyrill
// Stachniss.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "KinematicICP.hpp"

#include <Eigen/Core>
#include <kiss_icp/core/Preprocessing.hpp>
#include <kiss_icp/core/VoxelHashMap.hpp>
#include <vector>

namespace {
auto transform_points(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &pose) {
    std::vector<Eigen::Vector3d> points_transformed(points.size());
    std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
                   [&](const auto &point) { return pose * point; });
    return points_transformed;
}

auto Voxelize(const std::vector<Eigen::Vector3d> &frame, const double voxel_size) {
    const std::vector<Eigen::Vector3d> &frame_downsample =
        kiss_icp::VoxelDownsample(frame, voxel_size * 0.5);
    const std::vector<Eigen::Vector3d> &source =
        kiss_icp::VoxelDownsample(frame_downsample, voxel_size * 1.5);
    return std::make_tuple(source, frame_downsample);
}
}  // namespace
namespace kinematic_icp::pipeline {

KinematicICP::Vector3dVectorTuple KinematicICP::RegisterFrame(
    const std::vector<Eigen::Vector3d> &frame,
    const std::vector<double> &timestamps,
    const Sophus::SE3d &lidar_to_base,
    const Sophus::SE3d &relative_odometry,
    const bool free_mode,
    const int free_mode_max_iterations) {
    // Need to deskew in lidar frame
    const Sophus::SE3d &relative_odometry_in_lidar =
        lidar_to_base.inverse() * relative_odometry * lidar_to_base;
    const auto &preprocessed_frame =
        preprocessor_.Preprocess(frame, timestamps, relative_odometry_in_lidar);
    // Give the frame in base frame
    const auto &preprocessed_frame_in_base = transform_points(preprocessed_frame, lidar_to_base);
    // Voxelize
    // Localization patch (2026_IFAC): the source cascade may use a finer,
    // source-only voxel size (see Config::source_voxel_size).
    const double source_voxel_size =
        config_.source_voxel_size > 0.0 ? config_.source_voxel_size : config_.voxel_size;
    const auto &[source, frame_downsample] =
        Voxelize(preprocessed_frame_in_base, source_voxel_size);

    // Get adaptive_threshold
    const double tau = correspondence_threshold_.ComputeThreshold();

    // Run ICP
    const auto new_pose = registration_.ComputeRobotMotion(source,             // frame
                                                           local_map_,         // voxel_map
                                                           last_pose_,         // last_pose
                                                           relative_odometry,  // robot_motion
                                                           tau,   // max_correspondence_dist
                                                           free_mode, free_mode_max_iterations);

    // Compute the difference between the prediction and the actual estimate
    const auto odometry_error = (last_pose_ * relative_odometry).inverse() * new_pose;

    // Update step: threshold, local map and the last pose
    correspondence_threshold_.UpdateOdometryError(odometry_error);
    // Localization patch (2026_IFAC): skip local map update when running in
    // frozen-map localization mode (see Config::freeze_local_map).
    if (!config_.freeze_local_map) local_map_.Update(frame_downsample, new_pose);
    last_pose_ = new_pose;

    // Return the (deskew) input raw scan (frame) and the points used for
    // registration (source)
    return {preprocessed_frame_in_base, source};
}
}  // namespace kinematic_icp::pipeline
