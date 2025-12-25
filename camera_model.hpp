#pragma once
namespace visual_slam_2d {
class CameraModel {
    public:

    CameraModel(float f, int w) : focal_length_(f), width_(w) {
        
    }
    // Helper functions for reprojection
    float project(float x, float y) {
        // project the 2D landmark x and y into the 2D camera image.
        // The camera coordinate is defined such that y axis points forward and x axis points to the right.
        float u = x * focal_length_ / y;
        return u;
    }

    private:
    float focal_length_;
    float width_;

};
}  // namespace visual_slam_2d