#include <gtest/gtest.h>
#include "robot.hpp"
#include "camera_sensor.hpp"
#include "camera_model.hpp"
#include "localizer.hpp"

using namespace visual_slam_2d;

TEST(LocalizerTest, GaussNewtonConverges) {
    // Ground truth robot pose
    Pose2D gt_pose = makePose2D(1.0f, 2.0f, M_PI/6); 

    Robot robot;

    // Camera 1 unit forward of robot
    Pose2D cam_pose_rel = makePose2D(1.0f, 0.0f, 0.0f);
    CameraModel cam_model(2.0f, 640);
    auto camera = std::make_unique<CameraSensor>(cam_pose_rel, cam_model);
    robot.addCamera(std::move(camera));

    // Landmarks in world coordinates
    std::vector<LandmarkObservation> observations;
    std::vector<Eigen::Vector2f> landmarks_world = {
        {2.0f, 3.0f}, {0.0f, 4.0f}, {5.0f, 20.0f}
    };

    // std::vector<Eigen::Vector2f> landmarks_world = {
    //     // {2.0f, 3.0f}, {0.0f, 4.0f}, {3.0f, 4.0f}
    //     {2.0f, 3.0f}, {0.0f, 4.0f}, {3.0f, 2.0f}
    // };

    // Simulate measurements from ground-truth pose
    for (size_t i=0; i<landmarks_world.size(); ++i) {
        Pose2D cam_world = gt_pose * robot.cameras()[0]->extrinsic();
        Pose2D cam_T_world = cam_world.inverse();
        Eigen::Vector2f landmark_cam = cam_T_world * landmarks_world[i];
        float u;
        bool valid = robot.cameras()[0]->model().projectAndCheck(landmark_cam.x(), landmark_cam.y(), u);
        std::cout << "measurement " << landmark_cam.x() << " " << landmark_cam.y() << std::endl;
        if (valid) {
            std::cout << "Valid measurement with u " << u << std::endl;
            observations.push_back({landmarks_world[i], u, 0});
        }
        else {
            std::cout << "invalid " << u << std::endl;
        }
    }
    // Pose2D gt_pose = makePose2D(1.0f, 2.0f, M_PI/6); 

    // Initialize robot with wrong pose
    // robot.setPose(makePose2D(1.0f, 2.0f, 0.0f));
    // robot.setPose(makePose2D(1.0f, 2.0f, 0.0f));
    // Pose2D prior_pose = makePose2D(1.0f, 2.0f, M_PI/5); 
    // Pose2D prior_pose = makePose2D(0.9f, 2.1f, M_PI/5); 
    Pose2D prior_pose = makePose2D(0.9f, 2.1f, M_PI/4.8); 

    robot.setPose(prior_pose);

    Localizer localizer(robot);

    // Optimize
    localizer.optimizePose(observations, 20, 1e-6f);

    Pose2D optimized = robot.pose();

    // Check if optimized pose is close to ground truth
    EXPECT_NEAR(optimized.translation().x(), gt_pose.translation().x(), 1e-2f);
    EXPECT_NEAR(optimized.translation().y(), gt_pose.translation().y(), 1e-2f);
    EXPECT_NEAR(optimized.so2().log(), gt_pose.so2().log(), 1e-2f);
    // EXPECT_NEAR(1, 2, 1e-1f);
}
