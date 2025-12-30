#include <gtest/gtest.h>
#include "robot.hpp"
#include "camera_sensor.hpp"
#include "camera_model.hpp"
#include "landmark.hpp"

using namespace visual_slam_2d;

// What needs to be done for the extrinsic transforms?

TEST(RobotSensorTest, LandmarkProjection) {
    // tx, ty, theta
    // Pose2D world_T_robot = Pose2D::exp(Eigen::Vector3f(1.0f, 2.0f, M_PI/10)); 
    Pose2D world_T_robot = makePose2D(1.0f, 2.0f, M_PI/10); 
    Robot robot;
    robot.setPose(world_T_robot);

    // Make camera sensors to the robot
    Pose2D robot_T_camera = makePose2D(0.0f, 0.0f, -M_PI/10); 

    CameraModel cam_model(1.5f, 640);
    std::unique_ptr< CameraSensor> cam_sensor = std::make_unique<CameraSensor>(robot_T_camera, cam_model);

    robot.addCamera(std::move(cam_sensor));

    
    Landmark landmark(5.0f, 5.0f);

    Eigen::Vector2f landmark_wrt_world(5.0f, 5.0f);
    std::cout << "robot pose " << world_T_robot.matrix() << std::endl;
    Pose2D world_T_cam = world_T_robot * robot.cameras()[0]->extrinsic();
    std::cout << "world_T_cam " << world_T_cam.matrix() << std::endl;
    Eigen::Vector2f landmark_wrt_cam = world_T_cam.inverse() * landmark_wrt_world;
    std::cout << "Ldk wrt cam " << landmark_wrt_cam << std::endl;

    float projected = 0.0f;
    bool in_view = robot.cameras()[0]->model().projectAndCheck(landmark_wrt_cam.x(), landmark_wrt_cam.y(), projected);


    // Expected: landmark is at x=0, y=2 in camera frame
    EXPECT_NEAR(landmark_wrt_cam.x(), 4.0f, 1e-5f);
    EXPECT_NEAR(landmark_wrt_cam.y(), 3.0f, 1e-5f);

    // Projection: u = x * f / y = 0 * 2 / 2 = 0
    EXPECT_NEAR(projected, 2.0f, 1e-5f);
    EXPECT_TRUE(in_view);
}