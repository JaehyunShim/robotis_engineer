/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Read and Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is tested with a Dynamixel PRO 54-200, and an USB2DYNAMIXEL
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
//

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <dynamixel_sdk/dynamixel_sdk.h>              // Uses Dynamixel SDK library

#include <eigen3/Eigen/Eigen>
#include <max_kdl/max_kdl.h>

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64//562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116//596
#define ADDR_PRO_PRESENT_POSITION       132//611

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyACM0"      // Check which port is being used on your controller
// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      2300//-150000             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      2700//150000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10//20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

robotis_op::OP3KinematicsDynamics* kd_;

uint8_t dxl_id_[16];
uint8_t dxl_cnt_;

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

void headPositionCallback(const sensor_msgs::JointState::ConstPtr &msg)
{ 
  int dxl_cnt_ = 12;
  int dxl_goal_position[100] = {0, };							// Goal position
  //  int dxl_goal_position[dxl_cnt_] = {0, };		// Goal position
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position[100] = {0, };      // Present position

  ROS_INFO("Subscribed a topic!");

  // ?? device info
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler->openPort()){
    ROS_INFO("Succeeded to open the port!");
  } else{
    ROS_INFO("Failed to open the port!");
    ROS_INFO("Press any key to terminate...");
    getch();
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)){
    ROS_INFO("Succeeded to change the baudrate!");
  } else{
    ROS_INFO("Failed to change the baudrate!");
    ROS_INFO("Press any key to terminate...");
    getch();
    return;
  }
  
//  Eigen::MatrixXd Pos(3, 1);
  // Eigen::MatrixXd Ori(3, 1);
//  Pos << 0.093, 0.0, -0.05;
  // Pos << 0.09716, 0.0, -0.0685;
  // Ori << 0.0, 1.0, 0.0;

// Find Route
  kd_ = new robotis_op::OP3KinematicsDynamics(robotis_op::WholeBody);
  std::vector<int> idx;
  idx = kd_->findRoute(0, 19);
  for (int index = 0; index < idx.size() ; index++)
    ROS_INFO("%d", idx[index]);

  // kd_->calcForwardKinematics(0);
  
  double leg_angle[4];
  // if (kd_->computeIK(&leg_angle[0], 0, 0, -212.0*0.001, 0, 0, 0) == false)
  if (kd_->calcInverseKinematicsForLeftLeg(&leg_angle[0], 0.0*0.001, 20.0*0.001, -190.0*0.001, 0, 0, 0) == false)
    ROS_INFO("whahahthath");
  
  for (int index = 0; index < 4; index++)
    ROS_INFO("%f", leg_angle[index]);


  for (int index = 0; index < 4; index++){
    leg_angle[index] *= 4096 / (3.141593*2);
    ROS_INFO("%f", leg_angle[index]);
  }
  

  dxl_goal_position[0] = msg->position.at(0);
  dxl_goal_position[1] = msg->position.at(0);
  dxl_goal_position[2] = msg->position.at(0);
  dxl_goal_position[3] = msg->position.at(0);
  dxl_goal_position[4] = msg->position.at(0) + leg_angle[0];
  dxl_goal_position[5] = msg->position.at(0) + leg_angle[1];
  dxl_goal_position[6] = msg->position.at(0) + leg_angle[0];
  dxl_goal_position[7] = msg->position.at(0) + leg_angle[1];
  dxl_goal_position[8] = msg->position.at(0) + leg_angle[2];
  dxl_goal_position[9] = msg->position.at(0) + leg_angle[3];
  dxl_goal_position[10] = msg->position.at(0) + leg_angle[2];
  dxl_goal_position[11] = msg->position.at(0) + leg_angle[3];

  for (int index = 0; index < dxl_cnt_; index++){
    //    ROS_INFO("%f", msg->position.at(index));
    //    dxl_goal_position[index] = msg->position.at(index);
    int DXL_ID_ = index+1;
    //    int DXL_ID_ = idx[index];

    // Enable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
      ROS_INFO("%s", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0){
      ROS_INFO("%s", packetHandler->getRxPacketError(dxl_error));
    } else{
      ROS_INFO("Dynamixel has been successfully connected");
    }


    // Write Goal Position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID_, ADDR_PRO_GOAL_POSITION, dxl_goal_position[DXL_ID_-1], &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
      ROS_INFO("%s", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0){
      ROS_INFO("%s", packetHandler->getRxPacketError(dxl_error));
    }

    // Read Present Position
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID_, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position[DXL_ID_-1], &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
      ROS_INFO("%s", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0){
      ROS_INFO("%s", packetHandler->getRxPacketError(dxl_error));
    }
    ROS_INFO("[ID:%03d] GoalPos:%03d  PresPos:%03d", DXL_ID_, dxl_goal_position[DXL_ID_-1], dxl_present_position[DXL_ID_-1]);
  }

/*
  kd_->calcForwardKinematics(0); 
  Eigen::MatrixXd link_pose(3, 1);
  link_pose = kd_ -> op3_link_data_[idx[0]] -> position_;
  link_pose = kd_ -> op3_link_data_[idx[0]] -> position_;
  for (int i = 0; i < 3; i++)
    ROS_INFO("%f", link_pose(i));
*/

  return;
}

int main(int argc, char **argv)
{
  // Init ROS Node
  ros::init(argc, argv, "head_control");
  ros::NodeHandle nh_;
  ros::Subscriber head_position_sub_;




// Find Route
  kd_ = new robotis_op::OP3KinematicsDynamics(robotis_op::WholeBody);
  std::vector<int> idx;
  idx = kd_->findRoute(0, 19);
  for (int index = 0; index < idx.size() ; index++)
    ROS_INFO("%d", idx[index]);

  // kd_->calcForwardKinematics(0);
  
  double leg_angle[4];
  // if (kd_->computeIK(&leg_angle[0], 0, 0, -212.0*0.001, 0, 0, 0) == false)
  if (kd_->computeIK(&leg_angle[0], 0.0*0.001, 0.0*0.001, -190.0*0.001, 0, 0, 0) == false)
    ROS_INFO("whahahthath");
  
  for (int index = 0; index < 4; index++)
    ROS_INFO("%f", leg_angle[index]);


  if (kd_->calcInverseKinematicsForLeftLeg(&leg_angle[0], 0.0*0.001, 0.0*0.001, -190.0*0.001, 0, 0, 0) == false)
    ROS_INFO("whahahthath");
  
  for (int index = 0; index < 4; index++)
    ROS_INFO("%f", leg_angle[index]);





  // Subscribe Topics
  head_position_sub_ = nh_.subscribe("/head_position", 10, &headPositionCallback);
  ROS_INFO("Ready to subscribe topics");

  ros::spin();

  return 0;
}
