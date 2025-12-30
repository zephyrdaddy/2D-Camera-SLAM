#pragma once

#include <vector>
#include <memory>
#include "pose2d.hpp"
#include "camera_sensor.hpp"

namespace visual_slam_2d {

class Robot {
public:
    Robot() : world_T_robot(Pose2D()) {}

    const Pose2D& pose() const {
        return world_T_robot;
    }

    void setPose(const Pose2D& pose) {
        world_T_robot = pose;
    }

    void move(const Pose2D& delta) {
        world_T_robot = world_T_robot * delta;
    }

    void addCamera(std::unique_ptr<CameraSensor> camera) {
        cameras_.push_back(std::move(camera));
    }

    const std::vector<std::unique_ptr<CameraSensor>>& cameras() const {
        return cameras_;
    }

private:
    Pose2D world_T_robot;
    std::vector<std::unique_ptr<CameraSensor>> cameras_;
};

}  // namespace visual_slam_2d
