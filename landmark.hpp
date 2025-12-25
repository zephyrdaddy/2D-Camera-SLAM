#pragma once

namespace visual_slam_2d {
class Landmark {
    public:
    Landmark(float x, float y) : x_(x), y_(y) {

    }
    float x() {
        return x_;
    }

    float y() {
        return y_;
    }
    private:
    float x_;
    float y_;
    // Each landmark has a unique id.


};
}  // namespace visual_slam_2d