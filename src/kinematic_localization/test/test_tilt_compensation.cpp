#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "utils.hpp"

namespace {

using kinematic_localization::utils::LevelScan;
using kinematic_localization::utils::TiltParams;
using kinematic_localization::utils::TiltResult;

std::vector<Eigen::Vector3d> PlanarPoints() {
    std::vector<Eigen::Vector3d> points;
    for (int i = 0; i < 12; ++i) points.emplace_back(0.1 * i, 1.0, 0.0);
    return points;
}

TEST(TiltCompensation, ZeroGradientPreservesPrototypeInput) {
    const auto points = PlanarPoints();
    TiltParams config;
    TiltResult result;

    const auto corrected = LevelScan(points, Sophus::SE3d{}, 8.0, -3.0, config, &result);

    ASSERT_EQ(corrected.size(), points.size());
    for (size_t i = 0; i < points.size(); ++i) EXPECT_TRUE(corrected[i].isApprox(points[i], 0.0));
    EXPECT_DOUBLE_EQ(result.roll_rad, 0.0);
    EXPECT_DOUBLE_EQ(result.pitch_rad, 0.0);
    EXPECT_EQ(result.dropped, 0U);
}

TEST(TiltCompensation, AppliesAndClampsRollInBaseAxes) {
    const auto points = PlanarPoints();
    TiltParams config;
    config.roll_gradient_rad_per_mps2 = 0.1;
    config.max_angle_rad = 0.2;
    TiltResult result;

    const auto corrected = LevelScan(points, Sophus::SE3d{}, 4.0, 0.0, config, &result);

    ASSERT_EQ(corrected.size(), points.size());
    EXPECT_NEAR(result.roll_rad, 0.2, 1e-12);
    EXPECT_NEAR(corrected.front().y(), std::cos(0.2), 1e-12);
    EXPECT_DOUBLE_EQ(corrected.front().z(), 0.0);
}

TEST(TiltCompensation, RejectFallbackNeverStarvesIcp) {
    const auto points = PlanarPoints();
    TiltParams config;
    config.roll_gradient_rad_per_mps2 = 0.1;
    config.max_point_height_m = 0.01;
    TiltResult result;

    const auto corrected = LevelScan(points, Sophus::SE3d{}, 1.0, 0.0, config, &result);

    EXPECT_EQ(result.dropped, points.size());
    ASSERT_EQ(corrected.size(), points.size());
    for (size_t i = 0; i < points.size(); ++i) EXPECT_TRUE(corrected[i].isApprox(points[i], 0.0));
}

}  // namespace
