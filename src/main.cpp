// Copyright (c) 2023 WinGs Robotics
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <random>

#include <franka/exception.h>
#include <franka/robot.h>
#include <Eigen/Dense>

#include "examples_common.h"

#include "Motion_plan.h"
#include "PD_controller.h"
#include "kinematics.h"

const int Traj_HZ = 50;

std::vector<std::array<double, 16>> circle_motion(const std::array<double, 16> &initial_pose, const double &time_)
{
  double cx, cy, cz, cRx, cRy, cRz;
  matrixToPose(initial_pose, cx, cy, cz, cRx, cRy, cRz);
  std::array<double, 6> start_point = {cx, cy, cz, cRx, cRy, cRz};

  // Initialize random number generator with a seed
  std::mt19937 rng(std::random_device{}());

  // Create a uniform distribution from 0 to 99
  std::uniform_int_distribution<int> dist(0, 99);

  int n = Traj_HZ * time_;
  std::vector<std::vector<double>> trajectory(n);
  std::vector<std::array<double, 16>> traj;

  double delta_theta = 2.0 * M_PI / static_cast<double>(n);
  double theta = 0.0;
  double r = 0.1;
  for (int i = 0; i < n; i++)
  {
    double x = cx + r * std::sin(theta);     //+ dist(rng) * 0.00001;
    double z = cz + r * std::cos(theta) - r; //+ dist(rng) * 0.00001;
    trajectory[i] = {x, cy, z, cRx, cRy, cRz};
    theta += delta_theta;
  }

  traj = trajactory_generator(trajectory);
  return traj;
}

int main()
{
  // Parameters
  const size_t joint_number{3};
  const size_t filter_size{5};

  // NOLINTNEXTLINE(readability-identifier-naming)
  const std::array<double, 7> K_P{{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}};
  // NOLINTNEXTLINE(readability-identifier-naming)
  const std::array<double, 7> K_D{{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};

  Controller controller(filter_size, K_P, K_D);

  try
  {
    franka::Robot robot("172.16.0.2");
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    franka::RobotState state = robot.readOnce();
    std::vector<std::array<double, 16>> motion;
    std::array<double, 16> initial_pose;

    initial_pose = state.O_T_EE_c;

    std::cout << "Position Now is:" << std::endl;
    printArray16(initial_pose);

    motion = circle_motion(initial_pose, 30.0);
    plot_motion(initial_pose, motion);

    std::cin.ignore();

    int index = 0;
    double time = 0.0;
    robot.control([&](const franka::RobotState &robot_state, franka::Duration) -> franka::Torques
                  { return controller.step(robot_state); },

                  [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::JointPositions
                  {
                    time += period.toSec();
                    std::array<double, 16> new_pose;
                    std::array<double, 7> last_q;
                    new_pose = motion[index];
                    last_q = robot_state.q;

                    index = index + 1;
                    std::cout << time << " " << new_pose[12] << " " << new_pose[14] << std::endl;

                    Eigen::Matrix<double, 7, 1> last_q_mat;
                    last_q_mat = Eigen::Map<Eigen::Matrix<double, 7, 1>>(last_q.data());
                    Eigen::Matrix<double, 6, 1> new_pose_mat;
                    // todo() check new_pose how to convert to new_pose_mat implement in motion plan
                    Eigen::Matrix<double, 7, 1> new_angle = Kinematics::inverse(const Eigen::Matrix<double, 6, 1> &x_target, last_q_mat);

                    if (index >= motion.size())
                    {
                      std::cout << std::endl
                                << "Finished motion, shutting down example" << std::endl;
                      return franka::MotionFinished(new_angle);
                    }
                    return new_angle;
                  });
  }
  catch (const franka::Exception &e)
  {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
