#pragma once

#include "sensor.hpp"
#include "camera_model.hpp"

namespace visual_slam_2d {

class CameraSensor : public SensorBase {
public:
    CameraSensor(const Pose2D& robot_T_camera,
                 CameraModel camera)
        : SensorBase(robot_T_camera),
          camera_(std::move(camera)) {}

    const CameraModel& model() const {
        return camera_;
    }

private:
    CameraModel camera_;
};

}  // namespace visual_slam_2d
