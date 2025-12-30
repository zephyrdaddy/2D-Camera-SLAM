#pragma once
namespace visual_slam_2d {
class CameraModel {
    public:

    CameraModel(float f, int w) : focal_length_(f), width_(w), half_width_(width_ * 0.5f) {
    }
    // Helper functions for reprojection
    float project(float x, float y) const {
        // project the 2D landmark x and y into the 2D camera image.
        // The camera coordinate is defined such that y axis points forward and x axis points to the right.
        float u = x * focal_length_ / y;
        return u;
    }

    bool isInView(float projected) const {
        return projected >= -half_width_ && projected <= half_width_;
    }

    bool projectAndCheck(float x, float y, float& projected) const {
        if (y <= 0.0f) {
            return false;
        }
        projected = project(x, y);
        return isInView(projected);

    }

    float focal_length() const {
        return focal_length_;
    }

    private:
    const float focal_length_;
    const float width_;
    const float half_width_;

};
}  // namespace visual_slam_2d