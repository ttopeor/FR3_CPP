// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example generate_cartesian_pose_motion.cpp
 * An example showing how to generate a Cartesian motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */


std::array<double, 16> initial_pose;
double t = 0.0;

franka::CartesianPose func_callback(const franka::RobotState &robot_state, franka::Duration period)
{
  t += period.toSec();

  if (t == 0.0)
  {
    initial_pose = robot_state.O_T_EE_c;
  }

  double after_time = 5.0;
  double x_delta = 0.01 / after_time;
  double z_delta = 0.01 / after_time;
  std::array<double, 16> new_pose = initial_pose;

  new_pose[12] += x_delta * t;
  new_pose[14] += z_delta * t;

  if (int(t * 1000) % 1000 == 0)
  {
    std::cout << t << std::endl;
    std::cout << new_pose[12] << std::endl;
    std::cout << robot_state.O_T_EE[12]<< std::endl;
  }

  if (t >= after_time)
  {
    std::cout << std::endl
              << "Finished motion, shutting down example" << std::endl;
    return franka::MotionFinished(new_pose);
  }

  return new_pose;
}

int main()
{
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


    robot.control(func_callback);
  }
  catch (const franka::Exception &e)
  {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
