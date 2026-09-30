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
#pragma once

#include <Eigen/Core>
#include <kiss_icp/core/VoxelHashMap.hpp>
#include <sophus/se3.hpp>
#include <vector>

namespace kinematic_icp {

struct KinematicRegistration {
    explicit KinematicRegistration(const int max_num_iteration,
                                   const double convergence_criterion,
                                   const int max_num_threads,
                                   const bool use_adaptive_odometry_regularization,
                                   const double fixed_regularization,
                                   // Localization patch (2026_IFAC): soft lateral DoF
                                   const bool lateral_dof_enable = false,
                                   const double lateral_regularization_scale = 1.0,
                                   const double lateral_regularization_floor_tau2 = 1.0);

    // Localization patch (2026_IFAC) §8: `free_mode` drops every prior-side
    // constraint for this one frame — odometry regularization Omega, the
    // lateral regularization floor, the iteration cap and the convergence
    // early-exit. Used by the node in fast corners, where the measured pose
    // error doubles (7.7 cm / 0.72 deg below 0.3 rad/s -> 16.1 cm / 2.03 deg
    // above 0.8 rad/s).
    //
    // 🔴 Omega is what keeps JTJ invertible. With free_mode the solve is a
    // bare Gauss-Newton step: in a geometrically degenerate view (long
    // corridor, few walls) JTJ is singular and dx becomes non-finite. The
    // caller MUST keep the non-finite pose rollback in place — such frames
    // are dropped, not silently corrupted.
    Sophus::SE3d ComputeRobotMotion(const std::vector<Eigen::Vector3d> &frame,
                                    const kiss_icp::VoxelHashMap &voxel_map,
                                    const Sophus::SE3d &last_robot_pose,
                                    const Sophus::SE3d &relative_wheel_odometry,
                                    const double max_correspondence_distance,
                                    const bool free_mode = false,
                                    const int free_mode_max_iterations = 200);

    // Localization patch (2026_IFAC): registration quality diagnostics.
    // Filled by every ComputeRobotMotion call from values it already computes
    // (no extra data association). residual_rms / JTJ are taken at the entry
    // of the *final* iteration, i.e. one iteration stale — negligible when
    // converged, use `converged` + `final_dx_norm` to widen trust otherwise.
    struct Diagnostics {
        size_t num_correspondences = 0;  // last data association (0 = broken frame)
        size_t num_source_points = 0;
        double residual_rms = 0.0;
        double threshold_tau = 0.0;
        double beta = 0.0;      // odometry regularization weight actually used
        bool free_mode = false;  // §8: this frame ran with all priors dropped
        int iterations = 0;     // perturbation solves performed
        bool converged = false;  // left the loop via dx.norm() < criterion
        double final_dx_norm = 0.0;
        Eigen::MatrixXd JTJ;  // normalized (/N), pre-regularization; 2x2, 3x3 with lateral DoF
    };
    const Diagnostics &diagnostics() const { return diag_; }

    int max_num_iterations_;
    double convergence_criterion_;
    int max_num_threads_;
    bool use_adaptive_odometry_regularization_;
    double fixed_regularization_;
    // Localization patch (2026_IFAC): soft non-holonomic lateral DoF (§6)
    bool lateral_dof_enable_;
    double lateral_regularization_scale_;
    double lateral_regularization_floor_tau2_;
    Diagnostics diag_;
};
}  // namespace kinematic_icp
