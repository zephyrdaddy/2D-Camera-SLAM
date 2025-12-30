#pragma once
#include "pose2d.hpp"

namespace visual_slam_2d {

class SensorBase {
public:
    explicit SensorBase(const Pose2D& robot_T_sensor)
        : robot_T_sensor_(robot_T_sensor) {}

    virtual ~SensorBase() = default;

    const Pose2D& extrinsic() const {
        return robot_T_sensor_;
    }

protected:
    Pose2D robot_T_sensor_;
};

}  // namespace visual_slam_2d
