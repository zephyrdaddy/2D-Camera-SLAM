#pragma once
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

namespace visual_slam_2d {

using Pose2D = Sophus::SE2f;

inline Pose2D makePose2D(float x, float y, float theta) {
    return Pose2D(
        Sophus::SO2f(theta),
        Eigen::Vector2f(x, y)
    );
}

}  // namespace visual_slam_2d
