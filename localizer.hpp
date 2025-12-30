#pragma once

#include "robot.hpp"
#include "camera_sensor.hpp"
#include <vector>
#include <iostream>

namespace visual_slam_2d {

struct LandmarkObservation {
    Eigen::Vector2f world_position; // x,y in world
    float measured_u;               // observed projected value in camera
    int camera_index;               // which camera observed it
};

class Localizer {
public:
    Localizer(Robot& robot) : robot_(robot) {}

    void computeResidualsAndJacobian(const Pose2D& world_T_robot_prior,
                                     const std::vector<LandmarkObservation>& observations,
                                     Eigen::VectorXf& residuals,
                                     Eigen::MatrixXf& J) const
        {
        residuals.resize(observations.size());
        J.resize(observations.size(), 3);
        int obs_idx = 0;
        for (const auto& obs : observations) {
            const auto& cam = robot_.cameras()[obs.camera_index];
            Pose2D robot_T_camera = cam->extrinsic();
            Pose2D world_T_camera = world_T_robot_prior * robot_T_camera; // world_T_camera
            Pose2D cam_T_world = world_T_camera.inverse();
            Eigen::Vector2f landmark_cam_expected = cam_T_world * obs.world_position;
            Eigen::Vector2f landmark_robot_expected = world_T_robot_prior.inverse() * obs.world_position;

            float projected;
            bool in_view = cam->model().projectAndCheck(landmark_cam_expected.x(), landmark_cam_expected.y(), projected);
            if (!in_view) {
                std::cout << "Not in view  " << projected << std::endl;
                continue;
            }
            else {
                std::cout << "Yeahy " << projected << std::endl;
            }
            residuals(obs_idx) = projected - obs.measured_u;
            // Jacobian
            float x_c = landmark_cam_expected.x();
            float y_c = landmark_cam_expected.y();
            float x_r = landmark_robot_expected.x();
            float y_r = landmark_robot_expected.y();
            float f = cam->model().focal_length();
            // float y2 = y_c * y_c;
            Eigen::Matrix<float, 1, 2> J_proj;
            J_proj << (f / y_c), -(f * x_c) / (y_c * y_c) ;

            // Step B extrinsic rotation
            // This is simply the inverse rotation of the extrinsics
            Eigen::Matrix2f camera_R_robot = robot_T_camera.so2().inverse().matrix();

            // Step C robot motion jacobian (2x3)
            Eigen::Matrix<float, 2, 3> J_motion;
            J_motion << -1.0f, 0.0f, y_r, 
                         0.0f, -1.0f, -x_r;
                       Eigen::Matrix<float, 1, 3> J_combined = J_proj * camera_R_robot * J_motion;
            J(obs_idx, 0) = J_combined(0, 0);
            J(obs_idx, 1) = J_combined(0, 1);
            J(obs_idx, 2) = J_combined(0, 2);
            ++obs_idx;
        }
        residuals.resize(obs_idx);
        J.resize(obs_idx, 3);
    }

    // Simple Gauss-Newton step for robot pose
    void optimizePose(std::vector<LandmarkObservation>& observations,
                      int max_iterations = 10, float tol = 1e-5f)
    {
        Pose2D world_T_robot_initial = robot_.pose();
        Pose2D world_T_robot_prior = world_T_robot_initial;
        for (int iter = 0; iter < max_iterations; ++iter) {
            Eigen::VectorXf residuals;
            Eigen::MatrixXf J;
            computeResidualsAndJacobian(world_T_robot_prior, observations, residuals, J);

            if (residuals.size() == 0) break;

            // Gauss-Newton step
            Eigen::Vector3f delta = -(J.transpose() * J).ldlt().solve(J.transpose() * residuals);

            if (delta.norm() < tol) {
                break;
            }

            world_T_robot_prior = world_T_robot_prior * makePose2D(delta.x(), delta.y(), delta.z());
        }
        robot_.setPose(world_T_robot_prior); 
    }

private:
    Robot& robot_;
};

}  // namespace visual_slam_2d
