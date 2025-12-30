#pragma once

#include <cstdint>
#include <Eigen/Core>

namespace visual_slam_2d {

class Landmark {
public:
    // Constructor with explicit id and position components
    Landmark(std::uint32_t id, float x, float y)
        : id_(id), pos_(x, y)
    {}

    // Constructor with id and Eigen vector
    Landmark(std::uint32_t id, const Eigen::Vector2f& pos)
        : id_(id), pos_(pos)
    {}

    std::uint32_t id() const {
        return id_;
    }

    const Eigen::Vector2f& position() const {
        return pos_;
    }

    float x() const {
        return pos_.x();
    }

    float y() const {
        return pos_.y();
    }

private:
    std::uint32_t id_;     // Each landmark has a unique id.
    Eigen::Vector2f pos_;  // 2D position in world coordinates.
};

}  // namespace visual_slam_2d
