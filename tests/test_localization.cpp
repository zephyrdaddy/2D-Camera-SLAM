#include <gtest/gtest.h>
#include "robot.hpp"
#include "camera_sensor.hpp"
#include "camera_model.hpp"
#include "localizer.hpp"
#include "world.hpp"
#include "landmark.hpp"
#include "navlog.hpp"

using namespace visual_slam_2d;

TEST(LocalizerTest, GaussNewtonConverges) {
    // Ground truth robot pose
    Pose2D gt_pose = makePose2D(1.0f, 2.0f, M_PI / 6.0f);

    Robot robot;

    // Camera 1 unit forward of robot
    Pose2D cam_pose_rel = makePose2D(1.0f, 0.0f, 0.0f);
    CameraModel cam_model(2.0f, 640);
    auto camera = std::make_unique<CameraSensor>(cam_pose_rel, cam_model);
    robot.addCamera(std::move(camera));

    // World + landmarks (for metadata)
    World world;
    std::vector<Eigen::Vector2f> landmarks_world = {
        {2.0f, 3.0f}, {0.0f, 4.0f}, {5.0f, 20.0f}
    };
    for (std::uint32_t i = 0; i < landmarks_world.size(); ++i) {
        auto lm = std::make_shared<Landmark>(i, landmarks_world[i]);
        world.addLandMark(lm);
    }

    // Create logger
    // You may want a dedicated test_logs dir; adapt path as needed.
    NavlogWriter logger("test_logs/localizer_gauss_newton.navlog");

    // Write static metadata (world origin = identity)
    Pose2D world_origin = makePose2D(0.0f, 0.0f, 0.0f);
    logger.writeMetadata(world_origin, robot, world);

    // Simulate measurements from ground-truth pose
    std::vector<LandmarkObservation> observations;
    double t = 0.0;
    const double dt = 1.0;

    for (size_t i = 0; i < landmarks_world.size(); ++i) {
        Pose2D cam_world = gt_pose * robot.cameras()[0]->extrinsic();
        Pose2D cam_T_world = cam_world.inverse();
        Eigen::Vector2f landmark_cam = cam_T_world * landmarks_world[i];

        float u;
        bool valid = robot.cameras()[0]->model().projectAndCheck(
            landmark_cam.x(), landmark_cam.y(), u);

        if (valid) {
            observations.push_back({landmarks_world[i], u, 0});

            // Log observation into navlog
            NavlogObservationEvent obs_evt{};
            obs_evt.landmark_id  = static_cast<std::uint32_t>(i);
            obs_evt.camera_index = 0;
            obs_evt.u            = u;
            obs_evt.valid        = 1;
            logger.logObservation(t, obs_evt);
        }

        t += dt;
    }

    // Initialize robot with wrong pose
    Pose2D prior_pose = makePose2D(0.9f, 2.1f, M_PI / 4.8f);
    robot.setPose(prior_pose);

    Localizer localizer(robot);

    // Optionally log prior estimate
    {
        NavlogEstimateEvent est{};
        est.x = prior_pose.translation().x();
        est.y = prior_pose.translation().y();
        est.theta = prior_pose.so2().log();
        est.role_flags = 0b00000001; // prior
        logger.logEstimate(t, est);
        t += dt;
    }

    // Optimize
    localizer.optimizePose(observations, 20, 1e-6f);

    Pose2D optimized = robot.pose();

    // Log final optimized estimate
    {
        NavlogEstimateEvent est{};
        est.x = optimized.translation().x();
        est.y = optimized.translation().y();
        est.theta = optimized.so2().log();
        est.role_flags = 0b00000100; // final
        logger.logEstimate(t, est);
    }

    logger.flush();

    // Check if optimized pose is close to ground truth
    EXPECT_NEAR(optimized.translation().x(), gt_pose.translation().x(), 1e-2f);
    EXPECT_NEAR(optimized.translation().y(), gt_pose.translation().y(), 1e-2f);
    EXPECT_NEAR(optimized.so2().log(), gt_pose.so2().log(), 1e-2f);
}
