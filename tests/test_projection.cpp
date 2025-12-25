#include <gtest/gtest.h>
#include "camera_model.hpp"

using visual_slam_2d::CameraModel;

TEST(CameraModelTest, ProjectsCenterPointToZero) {
    CameraModel cam(100.0f, 640);
    EXPECT_FLOAT_EQ(cam.project(0.0f, 5.0f), 0.0f);
}

TEST(CameraModelTest, ProjectsCorrectly) {
    CameraModel cam(100.0f, 640);
    EXPECT_FLOAT_EQ(cam.project(2.0f, 4.0f), 50.0f);
}

TEST(CameraModelTest, ScaleInvariance) {
    CameraModel cam(100.0f, 640);
    EXPECT_FLOAT_EQ(
        cam.project(1.0f, 2.0f),
        cam.project(2.0f, 4.0f)
    );
}

TEST(CameraModelTest, LeftRightSymmetry) {
    CameraModel cam(100.0f, 640);
    EXPECT_FLOAT_EQ(
        cam.project(1.0f, 2.0f),
        -cam.project(-1.0f, 2.0f)
    );
}
