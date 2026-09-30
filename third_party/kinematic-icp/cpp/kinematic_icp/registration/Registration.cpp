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
#include "Registration.hpp"

#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/info.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/task_arena.h>

#include <algorithm>
#include <cmath>
#include <kiss_icp/core/VoxelHashMap.hpp>
#include <limits>
#include <numeric>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <tuple>

using Correspondences = tbb::concurrent_vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;

namespace {
constexpr double epsilon = std::numeric_limits<double>::min();

double ComputeOdometryRegularization(const Correspondences &associations,
                                     const Sophus::SE3d &odometry_initial_guess) {
    const double sum_of_squared_residuals =
        std::transform_reduce(associations.cbegin(), associations.cend(), 0.0, std::plus<double>(),
                              [&](const auto &association) {
                                  const auto &[source, target] = association;
                                  return (odometry_initial_guess * source - target).squaredNorm();
                              });
    const double N = static_cast<double>(associations.size());
    const double mean_squared_residual = sum_of_squared_residuals / N;
    const double beta = 1.0 / (mean_squared_residual + epsilon);
    return beta;
}

Correspondences DataAssociation(const std::vector<Eigen::Vector3d> &points,
                                const kiss_icp::VoxelHashMap &voxel_map,
                                const Sophus::SE3d &T,
                                const double max_correspondance_distance) {
    using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
    Correspondences correspondences;
    correspondences.reserve(points.size());
    tbb::parallel_for(
        // Range
        tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
        [&](const tbb::blocked_range<points_iterator> &r) {
            std::for_each(r.begin(), r.end(), [&](const auto &point) {
                const auto &[closest_neighbor, distance] = voxel_map.GetClosestNeighbor(T * point);
                if (distance < max_correspondance_distance) {
                    correspondences.emplace_back(point, closest_neighbor);
                }
            });
        });
    return correspondences;
}

// Localization patch (2026_IFAC): generalized over the perturbation dimension
// D so the soft lateral DoF (D = 3, see §6 of the robustness plan) reuses the
// same solver. D = 2 keeps the upstream arithmetic (same operations in the
// same order); the extra sum of squared residual norms feeds the registration
// diagnostics and does not touch the JTJ/JTr accumulation.
template <int D>
struct LinearSystem {
    Eigen::Matrix<double, D, D> JTJ = Eigen::Matrix<double, D, D>::Zero();
    Eigen::Matrix<double, D, 1> JTr = Eigen::Matrix<double, D, 1>::Zero();
    double squared_residual_sum = 0.0;
};

template <int D>
struct PerturbationResult {
    Eigen::Matrix<double, D, 1> dx = Eigen::Matrix<double, D, 1>::Zero();
    Eigen::Matrix<double, D, D> JTJ_normalized = Eigen::Matrix<double, D, D>::Zero();
    double squared_residual_mean = 0.0;
};

template <int D>
PerturbationResult<D> ComputePerturbation(const Correspondences &correspondences,
                                          const Sophus::SE3d &current_estimate,
                                          const Eigen::Matrix<double, D, D> &Omega) {
    auto compute_jacobian_and_residual = [&](const auto &correspondence) {
        const auto &[source, target] = correspondence;
        const Eigen::Vector3d residual = current_estimate * source - target;
        Eigen::Matrix<double, 3, D> J;
        if constexpr (D == 2) {
            J.col(0) = current_estimate.so3() * Eigen::Vector3d::UnitX();
            J.col(1) = current_estimate.so3() * Eigen::Vector3d(-source.y(), source.x(), 0.0);
        } else {
            J.col(0) = current_estimate.so3() * Eigen::Vector3d::UnitX();
            J.col(1) = current_estimate.so3() * Eigen::Vector3d::UnitY();  // lateral (§6)
            J.col(2) = current_estimate.so3() * Eigen::Vector3d(-source.y(), source.x(), 0.0);
        }
        return std::make_tuple(J, residual);
    };

    auto sum_linear_systems = [](LinearSystem<D> a, const LinearSystem<D> &b) {
        a.JTJ += b.JTJ;
        a.JTr += b.JTr;
        a.squared_residual_sum += b.squared_residual_sum;
        return a;
    };

    using correspondence_iterator = Correspondences::const_iterator;
    const LinearSystem<D> system = tbb::parallel_reduce(
        // Range
        tbb::blocked_range<correspondence_iterator>{correspondences.cbegin(),
                                                    correspondences.cend()},
        // Identity
        LinearSystem<D>{},
        // 1st Lambda: Parallel computation
        [&](const tbb::blocked_range<correspondence_iterator> &r,
            LinearSystem<D> J) -> LinearSystem<D> {
            return std::transform_reduce(
                r.begin(), r.end(), J, sum_linear_systems, [&](const auto &correspondence) {
                    const auto &[J_r, residual] = compute_jacobian_and_residual(correspondence);
                    return LinearSystem<D>{J_r.transpose() * J_r,        // JTJ
                                           J_r.transpose() * residual,   // JTr
                                           residual.squaredNorm()};
                });
        },
        // 2nd Lambda: Parallel reduction of the private Jacboians
        sum_linear_systems);
    const double num_correspondences = static_cast<double>(correspondences.size());

    PerturbationResult<D> result;
    result.JTJ_normalized = system.JTJ / num_correspondences;
    result.squared_residual_mean = system.squared_residual_sum / num_correspondences;
    Eigen::Matrix<double, D, D> JTJ = result.JTJ_normalized;
    const Eigen::Matrix<double, D, 1> JTr = system.JTr / num_correspondences;
    JTJ += Omega;
    result.dx = -(JTJ.inverse() * JTr);
    return result;
}

}  // namespace

namespace kinematic_icp {

KinematicRegistration::KinematicRegistration(const int max_num_iteration,
                                             const double convergence_criterion,
                                             const int max_num_threads,
                                             const bool use_adaptive_odometry_regularization,
                                             const double fixed_regularization,
                                             const bool lateral_dof_enable,
                                             const double lateral_regularization_scale,
                                             const double lateral_regularization_floor_tau2)
    : max_num_iterations_(max_num_iteration),
      convergence_criterion_(convergence_criterion),
      // Only manipulate the number of threads if the user specifies something
      // greater than 0
      max_num_threads_(max_num_threads > 0 ? max_num_threads
                                           : tbb::this_task_arena::max_concurrency()),
      use_adaptive_odometry_regularization_(use_adaptive_odometry_regularization),
      fixed_regularization_(fixed_regularization),
      // Localization patch (2026_IFAC): soft lateral DoF (§6), default off
      lateral_dof_enable_(lateral_dof_enable),
      lateral_regularization_scale_(lateral_regularization_scale),
      lateral_regularization_floor_tau2_(lateral_regularization_floor_tau2) {
    // This global variable requires static duration storage to be able to
    // manipulate the max concurrency from TBB across the entire class
    static const auto tbb_control_settings = tbb::global_control(
        tbb::global_control::max_allowed_parallelism, static_cast<size_t>(max_num_threads_));
}

Sophus::SE3d KinematicRegistration::ComputeRobotMotion(const std::vector<Eigen::Vector3d> &frame,
                                                       const kiss_icp::VoxelHashMap &voxel_map,
                                                       const Sophus::SE3d &last_robot_pose,
                                                       const Sophus::SE3d &relative_wheel_odometry,
                                                       const double max_correspondence_distance,
                                                       const bool free_mode,
                                                       const int free_mode_max_iterations) {
    Sophus::SE3d current_estimate = last_robot_pose * relative_wheel_odometry;
    // Localization patch (2026_IFAC): fresh per-frame diagnostics, also for the
    // early-out paths (a stale snapshot on exactly the broken frames would
    // poison the Mahalanobis gate that consumes these values).
    diag_ = Diagnostics{};
    diag_.num_source_points = frame.size();
    diag_.threshold_tau = max_correspondence_distance;
    if (voxel_map.Empty()) return current_estimate;

    auto motion_model = [](const Eigen::Vector2d &integrated_controls) {
        Sophus::SE3d::Tangent dx = Sophus::SE3d::Tangent::Zero();
        const double &displacement = integrated_controls(0);
        const double &theta = integrated_controls(1);
        dx(0) = displacement * std::sin(theta) / (theta + epsilon);
        dx(1) = displacement * (1.0 - std::cos(theta)) / (theta + epsilon);
        dx(5) = theta;
        return Sophus::SE3d::exp(dx);
    };
    // Localization patch (2026_IFAC): 3-DoF motion model for the soft lateral
    // DoF (§6) — the correction is a free SE(2) tangent instead of the
    // non-holonomic arc, the constraint moves into the regularizer Omega.
    auto motion_model_lateral = [](const Eigen::Vector3d &controls) {
        Sophus::SE3d::Tangent dx = Sophus::SE3d::Tangent::Zero();
        dx(0) = controls(0);
        dx(1) = controls(1);
        dx(5) = controls(2);
        return Sophus::SE3d::exp(dx);
    };
    auto correspondences =
        DataAssociation(frame, voxel_map, current_estimate, max_correspondence_distance);
    diag_.num_correspondences = correspondences.size();

    // Localization patch (2026_IFAC): no correspondences -> return the odometry
    // prediction without correction. The guard must sit above the
    // regularization lambda because it covers two NaN paths at once:
    //   1) ComputeOdometryRegularization: sum/N with N=0 -> NaN -> beta = NaN
    //      (the default use_adaptive_odometry_regularization path)
    //   2) ComputePerturbation: JTJ /= 0 -> NaN -> permanent last_pose_
    //      pollution
    if (correspondences.empty()) return current_estimate;

    const double regularization_term = [&]() {
        if (use_adaptive_odometry_regularization_) {
            return ComputeOdometryRegularization(correspondences, current_estimate);
        } else {
            return fixed_regularization_;
        }
    }();
    diag_.beta = free_mode ? 0.0 : regularization_term;
    diag_.free_mode = free_mode;
    // §8 free_mode: no cap-driven early stop, run until the step vanishes.
    const int iteration_budget = free_mode ? free_mode_max_iterations : max_num_iterations_;
    // ICP-loop
    if (!lateral_dof_enable_) {
        // Upstream 2-DoF path (longitudinal + yaw, non-holonomic arc) — the
        // arithmetic is bit-identical to upstream when the lateral DoF is off.
        const Eigen::Matrix2d Omega =
            free_mode ? Eigen::Matrix2d::Zero()
                      : Eigen::Matrix2d(Eigen::Vector2d(regularization_term, 0).asDiagonal());
        for (int j = 0; j < iteration_budget; ++j) {
            const auto solved = ComputePerturbation<2>(correspondences, current_estimate, Omega);
            const auto &dx = solved.dx;
            if (!dx.allFinite()) break;  // §8: Omega=0 -> JTJ can be singular
            diag_.iterations = j + 1;
            diag_.final_dx_norm = dx.norm();
            diag_.residual_rms = std::sqrt(solved.squared_residual_mean);
            diag_.JTJ = solved.JTJ_normalized;
            const auto delta_motion = motion_model(dx);
            current_estimate = current_estimate * delta_motion;
            // Break loop. §8 free_mode: the convergence gate is dropped; only a
            // numerically vanishing step (1e-9) stops the loop.
            if (dx.norm() < (free_mode ? 1e-9 : convergence_criterion_)) {
                diag_.converged = true;
                break;
            }
            correspondences =
                DataAssociation(frame, voxel_map, current_estimate, max_correspondence_distance);
            diag_.num_correspondences = correspondences.size();
            // Localization patch (2026_IFAC): stop when the re-association comes
            // back empty — the next ComputePerturbation would divide by zero.
            // break, not return: the recorded diagnostics of exactly these
            // frames are what the gate needs.
            if (correspondences.empty()) break;
        }
    } else {
        // Localization patch (2026_IFAC): soft lateral DoF (§6). The wheel
        // odometry's lateral information is exactly the no-side-slip prior, so
        // beta_lat = scale * beta is its soft version. MSR < tau^2 by
        // construction (DataAssociation only keeps distance < tau), hence
        // beta > 1/tau^2 — the floor is specified as a multiple of that
        // algebraic lower bound to keep the lateral DoF from opening fully on
        // opponent/map mismatch frames (large-MSR, small-beta).
        const double tau2 =
            max_correspondence_distance * max_correspondence_distance;
        const double beta_lat = std::max(lateral_regularization_floor_tau2_ / tau2,
                                         lateral_regularization_scale_ * regularization_term);
        const Eigen::Matrix3d Omega =
            free_mode ? Eigen::Matrix3d::Zero()
                      : Eigen::Matrix3d(
                            Eigen::Vector3d(regularization_term, beta_lat, 0).asDiagonal());
        for (int j = 0; j < iteration_budget; ++j) {
            const auto solved = ComputePerturbation<3>(correspondences, current_estimate, Omega);
            const auto &dx = solved.dx;
            if (!dx.allFinite()) break;  // §8: Omega=0 -> JTJ can be singular
            diag_.iterations = j + 1;
            diag_.final_dx_norm = dx.norm();
            diag_.residual_rms = std::sqrt(solved.squared_residual_mean);
            diag_.JTJ = solved.JTJ_normalized;
            const auto delta_motion = motion_model_lateral(dx);
            current_estimate = current_estimate * delta_motion;
            if (dx.norm() < (free_mode ? 1e-9 : convergence_criterion_)) {
                diag_.converged = true;
                break;
            }
            correspondences =
                DataAssociation(frame, voxel_map, current_estimate, max_correspondence_distance);
            diag_.num_correspondences = correspondences.size();
            if (correspondences.empty()) break;
        }
    }
    // Spit the final transformation
    return current_estimate;
}
}  // namespace kinematic_icp
