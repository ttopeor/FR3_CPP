// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>


#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
/**
 *
 * This is a Test
 *
 */

const int Traj_HZ = 50;


void printArray(std::array<double, 16> arr) {
    for (double element : arr) {
        std::cout << element << " ";
    }
    std::cout << std::endl;
}

void printVector(const std::vector<std::vector<double>>& vec) {
    for (int i = 0; i < vec.size(); i++) {
        for (int j = 0; j < vec[i].size(); j++) {
            std::cout << vec[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

void printVectorOfArrays(const std::vector<std::array<double, 16>>& vec)
{
    for (const auto& arr : vec) {  // iterate over each array in the vector
        for (const auto& val : arr) {  // iterate over each double value in the array
            std::cout << val << " ";   // print the value
        }
        std::cout << std::endl;  // move to the next line for the next array
    }
}

void matrixToPose(const std::array<double, 16>& matrix, double& x, double& y, double& z, double& Rx, double& Ry, double& Rz)
{
    // Extract translation components of matrix
    x = matrix[12];
    y = matrix[13];
    z = matrix[14];
    
    // Extract rotation components of matrix
    double sy = std::sqrt(matrix[0]*matrix[0] + matrix[1]*matrix[1]);
    bool singular = sy < 1e-6;
    if (!singular)
    {
        Rx = std::atan2(matrix[9], matrix[10]);
        Ry = std::atan2(-matrix[8], sy);
        Rz = std::atan2(matrix[4], matrix[0]);

        Rx = std::atan2(matrix[6], matrix[10]);
        Ry = std::atan2(-matrix[2], sy);
        Rz = std::atan2(matrix[1], matrix[0]);
    }else{   
      Rz = 0.0;
      if (abs(matrix[2] - (-1))<1e-6){ 
        Ry = M_PI/2; 
        Rx = Rz + atan2(matrix[4],matrix[8]);
      }else{ 
        Ry = -M_PI/2; 
        Rx = -1*Ry + atan2(-1*matrix[4],-1*matrix[8]); 
      }
    }
    
}

std::vector<std::array<double, 16>> traj_to_matrices(const std::vector<std::vector<double>>& traj) {
    std::vector<std::array<double, 16>> matrices;
    for (const auto& pose : traj) {
        std::array<double, 16> mat{};
        // Extract pose elements
        const double x = pose[0];
        const double y = pose[1];
        const double z = pose[2];
        const double rx = pose[3];
        const double ry = pose[4];
        const double rz = pose[5];

        // Compute transformation matrix
        const double ca = cos(rx);
        const double sa = sin(rx);
        const double cb = cos(ry);
        const double sb = sin(ry);
        const double cc = cos(rz);
        const double sc = sin(rz);

        mat[0] = cb*cc;
        mat[1] = sa*sb*sc+ca*sc;
        mat[2] = -ca*sb*cc+sa*sc;
        mat[3] = 0;
        mat[4] = -cb * sc;
        mat[5] = -sa * sb * sc + ca * cc;
        mat[6] = ca * sb * sc + sa * cc;
        mat[7] = 0;
        mat[8] = sb;
        mat[9] = -sa * cb;
        mat[10] = ca * cb;
        mat[11] = 0;
        mat[12] = x;
        mat[13] = y;
        mat[14] = z;
        mat[15] = 1;

        matrices.push_back(mat);
    }
    return matrices;
}
//input {{x,y,z,Rx,Ry,Rz},{x,y,z,Rx,Ry,Rz}........} 30HZ  --> output{T1,T2,T3,T4...........} 1000HZ
std::vector<std::array<double, 16>> trajactory_generator(const std::vector<std::vector<double>>& traj){
    std::vector<std::array<double, 16>> final_trajectory;
    //input check
    for (int i = 0; i < traj.size(); i++) {
        if (traj[i].size() != 6) {
            std::cout << std::endl
              << "Trajactory points not all 6dof, please check!" << std::endl;
            return final_trajectory;
        }
    }

    const int traj_size = traj.size();
    const double dtime1 = 1.0/Traj_HZ;
    const int slow_down_steps = 10;
    //smooth path by apply constant acceleration between points
    std::vector<std::vector<double>> traj_pos = traj;
    std::vector<std::vector<double>> traj_vel(traj_size, std::vector<double>(6));
    std::vector<std::vector<double>> traj_acc(traj_size+slow_down_steps, std::vector<double>(6));
    // calculate velocity and acceleration for each point
    for (int i = 0; i < traj_size-1; i++) {
      for(int j = 0; j < 6; j++){
        traj_acc[i][j] = 2*((traj_pos[i+1][j]-traj_pos[i][j])-traj_vel[i][j]*dtime1)/pow(dtime1, 2);
        traj_vel[i+1][j] = traj_acc[i][j] * dtime1 + traj_vel[i][j];
      }
    }

    

    for(int i = traj_size-1; i < traj_size+slow_down_steps; i++){
      for(int j = 0; j < 6; j++){
        traj_acc[i][j] = (0-traj_vel[traj_size-1][j])/(slow_down_steps*dtime1);
      }
    }


    double time = 0.0;
    int index =0;
    double dtime2 = 0.001;
    const int up_sample_times = int(1000.0/Traj_HZ);
    std::vector<std::vector<double>> generated_traj((traj_size+slow_down_steps)*up_sample_times, std::vector<double>(6));
    for(int i = 0; i < traj_size+slow_down_steps; i++){
      if (i<traj_size-1){
        for(int j = 0; j < up_sample_times; j++){
          for(int k = 0; k < 6; k++){
                generated_traj[index][k] = traj_pos[i][k]+0.5*traj_acc[i][k]*pow(time, 2)+traj_vel[i][k]*time;
          }
          for (int j = 0; j < generated_traj[index].size(); j++) {
            //std::cout << generated_traj[index][j] << " ";
          }
          //std::cout << std::endl;
          time = time + dtime2;
          index = index + 1;
        }
        time = 0.0;
      }else{
        for(int j = 0; j < up_sample_times; j++){
          for(int k = 0; k < 6; k++){
                generated_traj[index][k] = traj_pos[traj_size-1][k]+0.5*traj_acc[i][k]*pow(time, 2)+traj_vel[traj_size-1][k]*time;
          }
          //std::cout <<"start slow done"<< std::endl;
          for (int j = 0; j < generated_traj[index].size(); j++) {
            //std::cout << generated_traj[index][j] << " ";
          }
          //std::cout << std::endl;

          time = time + dtime2;
          index = index + 1;
        }      
      }
    }
    
   //transformation from {x,y,z,Rx,Ry,Rz} to T

   final_trajectory = traj_to_matrices(generated_traj);

   return final_trajectory;
    
}

std::vector<std::array<double, 16>> circle_motion(const std::array<double, 16>& initial_pose){
    double cx, cy, cz, cRx, cRy, cRz;
    matrixToPose(initial_pose, cx, cy, cz, cRx, cRy, cRz);
    std::array<double, 6> start_point = {cx, cy, cz, cRx, cRy, cRz};

    int n=Traj_HZ*30;
    std::vector<std::vector<double>> trajectory(n);
    std::vector<std::array<double, 16>> traj;
    
    double delta_theta = 2.0 * M_PI / static_cast<double>(n);
    double theta = 0.0;
    double r = 0.1;
    for (int i = 0; i < n; i++)
      {
        double x = cx + r * std::sin(theta);
        double z = cz + r * std::cos(theta)-r;
        trajectory[i] = { x, cy, z, cRx, cRy, cRz };
        theta += delta_theta;
      }
  
    traj = trajactory_generator(trajectory); 
    return traj;

}


int main_2()
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
    franka::RobotState state = robot.readOnce();
    std::vector<std::array<double, 16>> motion;
    std::array<double, 16>  initial_pose;
    
    initial_pose = state.O_T_EE_c;

    motion = circle_motion(initial_pose);

    for(int i = 0; i < 100; i++){
      printArray(motion[i]);
    }

    int index = 0;
    robot.control([&motion,&index,&initial_pose](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
 

      std::array<double, 16> new_pose;
      new_pose = motion[index];
      //printArray(motion[index]);
      //new_pose = initial_pose;
      index = index + 1;
      
      if (index >= motion.size()) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        printArray(initial_pose);
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
    });
    
  }
  catch (const franka::Exception &e)
  {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}

int main(){
  std::array<double, 16>  initial_position;
  std::vector<std::array<double, 16>> motion;

  initial_position = {0.99999,1.05474e-06,2.81911e-06,0,1.05474e-06,-0.99999,-7.06459e-08,0,2.81913e-06,7.06495e-08,-1,0,0.30695,7.10655e-07,0.487024,1 };
  motion = circle_motion(initial_position);

  for(int i = 0; i < 100; i++){
    printArray(motion[i]);
  }

  std::vector<double> x, y, z;
      for (const auto& pose : motion) {
          x.push_back(pose[12]);
          y.push_back(pose[13]);
          z.push_back(pose[14]);
      }

      plt::figure();

      // Create a 3D scatter plot of the motion data
      plt::scatter(x, z);
      plt::xlabel("x");
      plt::ylabel("z");
      // Show the plot
      plt::show();

      
  return 0;
}

