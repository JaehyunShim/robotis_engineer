/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
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

/* Author: SCH, Jay Song, Kayman, Ryan Shim */

#include <iostream>
#include "robotis_engineer_kdl/robotis_engineer_kdl.h"

namespace robotis_engineer
{

EngineerKinematicsDynamics::EngineerKinematicsDynamics()
{
}
EngineerKinematicsDynamics::~EngineerKinematicsDynamics()
{
}

EngineerKinematicsDynamics::EngineerKinematicsDynamics(TreeSelect tree)
{
  for (int id = 0; id <= ALL_JOINT_ID; id++)
    robotis_engineer_link_data_[id] = new LinkData();

  if (tree == WholeBody)
  {
    robotis_engineer_link_data_[0]->name_ = "base";
    robotis_engineer_link_data_[0]->parent_ = -1;
    robotis_engineer_link_data_[0]->sibling_ = -1;
    robotis_engineer_link_data_[0]->child_ = 1;
    robotis_engineer_link_data_[0]->mass_ = 0.0;
    robotis_engineer_link_data_[0]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    robotis_engineer_link_data_[0]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    robotis_engineer_link_data_[0]->joint_limit_max_ = 100.0;
    robotis_engineer_link_data_[0]->joint_limit_min_ = -100.0;

    /*----- right arm -----*/
    // right arm shoulder pitch
    robotis_engineer_link_data_[1]->name_ = "r_shoulder_pitch";
    robotis_engineer_link_data_[1]->parent_ = 0;
    robotis_engineer_link_data_[1]->sibling_ = 3;
    robotis_engineer_link_data_[1]->child_ = 2;
    robotis_engineer_link_data_[1]->mass_ = 0.0;
    robotis_engineer_link_data_[1]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0); // (-0.0714*cos(M_PI/12), 0.0714*sin(M_PI/12), 0)
    robotis_engineer_link_data_[1]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.9659, -0.2588);        // (0, cos(M_PI/12), -sin(M_PI/12))
    robotis_engineer_link_data_[1]->joint_limit_max_ = 0.5 * 180;
    robotis_engineer_link_data_[1]->joint_limit_min_ = -0.5 * 180;

    // right arm shoulder roll
    robotis_engineer_link_data_[2]->name_ = "r_shoulder_roll";
    robotis_engineer_link_data_[2]->parent_ = 1;
    robotis_engineer_link_data_[2]->sibling_ = -1;
    robotis_engineer_link_data_[2]->child_ = 13;
    robotis_engineer_link_data_[2]->mass_ = 0.0;
    robotis_engineer_link_data_[2]->relative_position_ = robotis_framework::getTransitionXYZ(-1.0, 0.0, 0.0);
    robotis_engineer_link_data_[2]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    robotis_engineer_link_data_[2]->joint_limit_max_ = 0.3 * 180;
    robotis_engineer_link_data_[2]->joint_limit_min_ = -0.5 * 180;

    // right arm end effector
    robotis_engineer_link_data_[13]->name_ = "r_endEffector";
    robotis_engineer_link_data_[13]->parent_ = 2;
    robotis_engineer_link_data_[13]->sibling_ = -1;
    robotis_engineer_link_data_[13]->child_ = -1;
    robotis_engineer_link_data_[13]->mass_ = 0.0;
    robotis_engineer_link_data_[13]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    robotis_engineer_link_data_[13]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    robotis_engineer_link_data_[13]->joint_limit_max_ = 0;
    robotis_engineer_link_data_[13]->joint_limit_min_ = 0;
  

    /*----- left arm -----*/
    // left arm shoulder pitch
    robotis_engineer_link_data_[3]->name_ = "l_shoulder_pitch";
    robotis_engineer_link_data_[3]->parent_ = 0;
    robotis_engineer_link_data_[3]->sibling_ = 5;
    robotis_engineer_link_data_[3]->child_ = 4;
    robotis_engineer_link_data_[3]->mass_ = 0.0;
    robotis_engineer_link_data_[3]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0); // (0, 0.0714*cos(M_PI/12), 0.0714*sin(M_PI/12))
    robotis_engineer_link_data_[3]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);        // (0, cos(M_PI/12), sin(M_PI/12))
    robotis_engineer_link_data_[3]->joint_limit_max_ = 100 * 180;
    robotis_engineer_link_data_[3]->joint_limit_min_ = -100 * 180;

    // left arm shoulder roll
    robotis_engineer_link_data_[4]->name_ = "l_shoulder_roll";
    robotis_engineer_link_data_[4]->parent_ = 3;
    robotis_engineer_link_data_[4]->sibling_ = -1;
    robotis_engineer_link_data_[4]->child_ = 14;
    robotis_engineer_link_data_[4]->mass_ = 0.0;
    robotis_engineer_link_data_[4]->relative_position_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    robotis_engineer_link_data_[4]->joint_axis_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    robotis_engineer_link_data_[4]->joint_limit_max_ = 100 * 180;
    robotis_engineer_link_data_[4]->joint_limit_min_ = -100 * 180;

    // left arm end effector
    robotis_engineer_link_data_[14]->name_ = "l_endEffector";
    robotis_engineer_link_data_[14]->parent_ = 4;
    robotis_engineer_link_data_[14]->sibling_ = -1;
    robotis_engineer_link_data_[14]->child_ = -1;
    robotis_engineer_link_data_[14]->mass_ = 0.0;
    robotis_engineer_link_data_[14]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    robotis_engineer_link_data_[14]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    robotis_engineer_link_data_[14]->joint_limit_max_ = 100 * 180;
    robotis_engineer_link_data_[14]->joint_limit_min_ = -100 * 180;


    /*----- right leg -----*/

    // right hip roll
    robotis_engineer_link_data_[5]->name_ = "r_hip_roll";
    robotis_engineer_link_data_[5]->parent_ = 0;
    robotis_engineer_link_data_[5]->sibling_ = 7;
    robotis_engineer_link_data_[5]->child_ = 6;
    robotis_engineer_link_data_[5]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, -0.03525, -0.08985);
    robotis_engineer_link_data_[5]->joint_axis_ = robotis_framework::getTransitionXYZ(-1.0, 0.0, 0.0);
    robotis_engineer_link_data_[5]->joint_limit_max_ = 0.5 * 180;
    robotis_engineer_link_data_[5]->joint_limit_min_ = -0.5 * 180;

    // right hip pitch
    robotis_engineer_link_data_[6]->name_ = "r_hip_pitch";
    robotis_engineer_link_data_[6]->parent_ = 5;
    robotis_engineer_link_data_[6]->sibling_ = -1;
    robotis_engineer_link_data_[6]->child_ = 15;
    robotis_engineer_link_data_[6]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.0240);
    robotis_engineer_link_data_[6]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    robotis_engineer_link_data_[6]->joint_limit_max_ = 0.5 * 180;
    robotis_engineer_link_data_[6]->joint_limit_min_ = -0.5 * 180;

    // right knee upper
    robotis_engineer_link_data_[15]->name_ = "r_knee_upper";
    robotis_engineer_link_data_[15]->parent_ = 6;
    robotis_engineer_link_data_[15]->sibling_ = -1;
    robotis_engineer_link_data_[15]->child_ = 16;
    robotis_engineer_link_data_[15]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.0600);
    robotis_engineer_link_data_[15]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    robotis_engineer_link_data_[15]->joint_limit_max_ = 0.5 * 180;
    robotis_engineer_link_data_[15]->joint_limit_min_ = -0.5 * 180;

    // right knee lower
    robotis_engineer_link_data_[16]->name_ = "r_knee_lower";
    robotis_engineer_link_data_[16]->parent_ = 15;
    robotis_engineer_link_data_[16]->sibling_ = -1;
    robotis_engineer_link_data_[16]->child_ = 9;
    robotis_engineer_link_data_[16]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.0155);
    robotis_engineer_link_data_[16]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    robotis_engineer_link_data_[16]->joint_limit_max_ = 0.5 * 180;
    robotis_engineer_link_data_[16]->joint_limit_min_ = -0.5 * 180;

    // right ankle pitch
    robotis_engineer_link_data_[9]->name_ = "r_ankle_pitch";
    robotis_engineer_link_data_[9]->parent_ = 16;
    robotis_engineer_link_data_[9]->sibling_ = -1;
    robotis_engineer_link_data_[9]->child_ = 10;
    robotis_engineer_link_data_[9]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.0600);
    robotis_engineer_link_data_[9]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    robotis_engineer_link_data_[9]->joint_limit_max_ = 0.5 * 180;
    robotis_engineer_link_data_[9]->joint_limit_min_ = -0.5 * 180;

    // right ankle roll
    robotis_engineer_link_data_[10]->name_ = "r_ankle_roll";
    robotis_engineer_link_data_[10]->parent_ = 9;
    robotis_engineer_link_data_[10]->sibling_ = -1;
    robotis_engineer_link_data_[10]->child_ = 19;
    robotis_engineer_link_data_[10]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.0240);
    robotis_engineer_link_data_[10]->joint_axis_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    robotis_engineer_link_data_[10]->joint_limit_max_ = 0.5 * 180;
    robotis_engineer_link_data_[10]->joint_limit_min_ = -0.5 * 180;

    // right sole
    robotis_engineer_link_data_[19]->name_ = "r_sole";
    robotis_engineer_link_data_[19]->parent_ = 10;
    robotis_engineer_link_data_[19]->sibling_ = -1;
    robotis_engineer_link_data_[19]->child_ = -1;
    robotis_engineer_link_data_[19]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.0285);
    robotis_engineer_link_data_[19]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    robotis_engineer_link_data_[19]->joint_limit_max_ = 0.5 * 180;
    robotis_engineer_link_data_[19]->joint_limit_min_ = -0.5 * 180;


    /*----- left leg -----*/
    // left hip roll
    robotis_engineer_link_data_[7]->name_ = "l_hip_roll";
    robotis_engineer_link_data_[7]->parent_ = 0;
    robotis_engineer_link_data_[7]->sibling_ = -1;
    robotis_engineer_link_data_[7]->child_ = 8;
    robotis_engineer_link_data_[7]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.03525, -0.08985);
    robotis_engineer_link_data_[7]->joint_axis_ = robotis_framework::getTransitionXYZ(-1.0, 0.0, 0.0);
    robotis_engineer_link_data_[7]->joint_limit_max_ = 0.5 * 180;
    robotis_engineer_link_data_[7]->joint_limit_min_ = -0.5 * 180;

    // left hip pitch
    robotis_engineer_link_data_[8]->name_ = "l_hip_pitch";
    robotis_engineer_link_data_[8]->parent_ = 7;
    robotis_engineer_link_data_[8]->sibling_ = -1;
    robotis_engineer_link_data_[8]->child_ = 17;
    robotis_engineer_link_data_[8]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.0240);
    robotis_engineer_link_data_[8]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    robotis_engineer_link_data_[8]->joint_limit_max_ = 0.5 * 180;
    robotis_engineer_link_data_[8]->joint_limit_min_ = -0.5 * 180;

    // right knee upper
    robotis_engineer_link_data_[17]->name_ = "r_knee_upper";
    robotis_engineer_link_data_[17]->parent_ = 8;
    robotis_engineer_link_data_[17]->sibling_ = -1;
    robotis_engineer_link_data_[17]->child_ = 18;
    robotis_engineer_link_data_[17]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.0600);
    robotis_engineer_link_data_[17]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    robotis_engineer_link_data_[17]->joint_limit_max_ = 0.5 * 180;
    robotis_engineer_link_data_[17]->joint_limit_min_ = -0.5 * 180;

    // right knee lower
    robotis_engineer_link_data_[18]->name_ = "r_knee_lower";
    robotis_engineer_link_data_[18]->parent_ = 17;
    robotis_engineer_link_data_[18]->sibling_ = -1;
    robotis_engineer_link_data_[18]->child_ = 11;
    robotis_engineer_link_data_[18]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.0155);
    robotis_engineer_link_data_[18]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    robotis_engineer_link_data_[18]->joint_limit_max_ = 0.5 * 180;
    robotis_engineer_link_data_[18]->joint_limit_min_ = -0.5 * 180;

    // left ankle pitch
    robotis_engineer_link_data_[11]->name_ = "l_ankle_pitch";
    robotis_engineer_link_data_[11]->parent_ = 18;
    robotis_engineer_link_data_[11]->sibling_ = -1;
    robotis_engineer_link_data_[11]->child_ = 12;
    robotis_engineer_link_data_[11]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.0600);
    robotis_engineer_link_data_[11]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    robotis_engineer_link_data_[11]->joint_limit_max_ = 0.5 * 180;
    robotis_engineer_link_data_[11]->joint_limit_min_ = -0.5 * 180;

    // left ankle roll
    robotis_engineer_link_data_[12]->name_ = "l_ankle_roll";
    robotis_engineer_link_data_[12]->parent_ = 11;
    robotis_engineer_link_data_[12]->sibling_ = -1;
    robotis_engineer_link_data_[12]->child_ = 20;
    robotis_engineer_link_data_[12]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.0240);
    robotis_engineer_link_data_[12]->joint_axis_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    robotis_engineer_link_data_[12]->joint_limit_max_ = 0.5 * 180;
    robotis_engineer_link_data_[12]->joint_limit_min_ = -0.5 * 180;

    // left sole
    robotis_engineer_link_data_[20]->name_ = "l_sole";
    robotis_engineer_link_data_[20]->parent_ = 12;
    robotis_engineer_link_data_[20]->sibling_ = -1;
    robotis_engineer_link_data_[20]->child_ = -1;
    robotis_engineer_link_data_[20]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.0285);
    robotis_engineer_link_data_[20]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    robotis_engineer_link_data_[20]->joint_limit_max_ = 0.0;
    robotis_engineer_link_data_[20]->joint_limit_min_ = 0.0;
  }

  thigh_length_m_ = std::fabs(robotis_engineer_link_data_[ID_R_LEG_START + 2 * 3]->relative_position_.coeff(2, 0));
  calf_length_m_ = std::fabs(robotis_engineer_link_data_[ID_R_LEG_START + 2 * 4]->relative_position_.coeff(2, 0));
  ankle_length_m_ = std::fabs(robotis_engineer_link_data_[ID_R_LEG_END]->relative_position_.coeff(2, 0));
  leg_side_offset_m_ = 2.0 * (std::fabs(robotis_engineer_link_data_[ID_R_LEG_START]->relative_position_.coeff(1, 0)));
}
// didnt consider to = 0
std::vector<int> EngineerKinematicsDynamics::findRoute(int to)  
{
  int id = robotis_engineer_link_data_[to]->parent_;

  std::vector<int> idx;                     //why not matrix but std::vector?

  if (id == 0)
  {
    idx.push_back(0);
    idx.push_back(to);
  }
  else
  {
    idx = findRoute(id);
    idx.push_back(to);
  }

  return idx;
}
// didnt consider from = to
std::vector<int> EngineerKinematicsDynamics::findRoute(int from, int to) 
{
  int id = robotis_engineer_link_data_[to]->parent_;

  std::vector<int> idx;

  if (id == from)
  {
    idx.push_back(from);
    idx.push_back(to);
  }
  else if (id != 0)
  {
    idx = findRoute(from, id);
    idx.push_back(to);
  }

  return idx;
}

double EngineerKinematicsDynamics::calcTotalMass(int joint_id)
{
  double mass;

  if (joint_id == -1)
    mass = 0.0;
  else
    mass = robotis_engineer_link_data_[joint_id]->mass_ 
        + calcTotalMass(robotis_engineer_link_data_[joint_id]->sibling_)
        + calcTotalMass(robotis_engineer_link_data_[joint_id]->child_);

  return mass;
}

Eigen::MatrixXd EngineerKinematicsDynamics::calcMC(int joint_id)
{
  Eigen::MatrixXd mc(3, 1);

  if (joint_id == -1)
    mc = Eigen::MatrixXd::Zero(3, 1);
  else
  {
    mc = robotis_engineer_link_data_[joint_id]->mass_
      * (robotis_engineer_link_data_[joint_id]->orientation_ 
        * robotis_engineer_link_data_[joint_id]->center_of_mass_
        + robotis_engineer_link_data_[joint_id]->position_);
    mc = mc 
      + calcMC(robotis_engineer_link_data_[joint_id]->sibling_) 
      + calcMC(robotis_engineer_link_data_[joint_id]->child_);
  }

  return mc;
}
// better write MC and M, using small letters cause misunderstanding.
Eigen::MatrixXd EngineerKinematicsDynamics::calcCOM(Eigen::MatrixXd mc)
{
  double mass;                      
  Eigen::MatrixXd COM(3, 1);

  mass = calcTotalMass(0);
  COM = mc / mass;

  return COM;
}

void EngineerKinematicsDynamics::calcForwardKinematics(int joint_id)
{
  if (joint_id == -1)
    return;

  if (joint_id == 0)
  {
    robotis_engineer_link_data_[0]->position_ = Eigen::MatrixXd::Zero(3, 1);
    robotis_engineer_link_data_[0]->orientation_ = robotis_framework::calcRodrigues(
        robotis_framework::calcHatto(robotis_engineer_link_data_[0]->joint_axis_),
                                    robotis_engineer_link_data_[0]->joint_angle_);
  }

  if (joint_id != 0)
  {
    int parent = robotis_engineer_link_data_[joint_id]->parent_;

    robotis_engineer_link_data_[joint_id]->position_ = robotis_engineer_link_data_[parent]->orientation_
        * robotis_engineer_link_data_[joint_id]->relative_position_ + robotis_engineer_link_data_[parent]->position_;
    robotis_engineer_link_data_[joint_id]->orientation_ = robotis_engineer_link_data_[parent]->orientation_
        * robotis_framework::calcRodrigues(robotis_framework::calcHatto(robotis_engineer_link_data_[joint_id]->joint_axis_),
                                           robotis_engineer_link_data_[joint_id]->joint_angle_);

    robotis_engineer_link_data_[joint_id]->transformation_.block<3, 1>(0, 3) = robotis_engineer_link_data_[joint_id]->position_;
    robotis_engineer_link_data_[joint_id]->transformation_.block<3, 3>(0, 0) = robotis_engineer_link_data_[joint_id]->orientation_;
  }

  ROS_INFO("Joint#: %d,  %f %f %f", joint_id, 
    robotis_engineer_link_data_[joint_id]->position_(0),
    robotis_engineer_link_data_[joint_id]->position_(1),
    robotis_engineer_link_data_[joint_id]->position_(2));

  calcForwardKinematics(robotis_engineer_link_data_[joint_id]->sibling_);
  calcForwardKinematics(robotis_engineer_link_data_[joint_id]->child_);
}

Eigen::MatrixXd EngineerKinematicsDynamics::calcJacobian(std::vector<int> idx)
{
  int idx_size = idx.size();
  int end = idx_size - 1;

  Eigen::MatrixXd tar_position = robotis_engineer_link_data_[idx[end]]->position_;
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, idx_size);

  for (int id = 0; id < idx_size; id++)
  {
    int curr_id = idx[id];

    Eigen::MatrixXd tar_orientation = robotis_engineer_link_data_[curr_id]->orientation_ * robotis_engineer_link_data_[curr_id]->joint_axis_;

    jacobian.block(0, id, 3, 1) = robotis_framework::calcCross(tar_orientation,
                                                               tar_position - robotis_engineer_link_data_[curr_id]->position_);
    jacobian.block(3, id, 3, 1) = tar_orientation;    // difference from the other one? about block function/?
  }

  return jacobian;
}

Eigen::MatrixXd EngineerKinematicsDynamics::calcJacobianCOM(std::vector<int> idx)
{
  int idx_size = idx.size();
  int end = idx_size - 1;

  Eigen::MatrixXd tar_position = robotis_engineer_link_data_[idx[end]]->position_;
  Eigen::MatrixXd jacobian_com = Eigen::MatrixXd::Zero(6, idx_size);

  for (int id = 0; id < idx_size; id++)
  {
    int curr_id = idx[id];
    double mass = calcTotalMass(curr_id);

    Eigen::MatrixXd og = calcMC(curr_id) / mass - robotis_engineer_link_data_[curr_id]->position_;  // COM - position   )   og stands for ??
    Eigen::MatrixXd tar_orientation = robotis_engineer_link_data_[curr_id]->orientation_ * robotis_engineer_link_data_[curr_id]->joint_axis_;

    jacobian_com.block(0, id, 3, 1) = robotis_framework::calcCross(tar_orientation, og); // target position is replaced with COM
    jacobian_com.block(3, id, 3, 1) = tar_orientation;
  }

  return jacobian_com;
}

Eigen::MatrixXd EngineerKinematicsDynamics::calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position,
                                                 Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation)
{
  Eigen::MatrixXd pos_err = tar_position - curr_position;
  Eigen::MatrixXd ori_err = curr_orientation.transpose() * tar_orientation;
  Eigen::MatrixXd ori_err_dash = curr_orientation * robotis_framework::convertRotToOmega(ori_err);  // better use angvel_err or angVel_err

  Eigen::MatrixXd err = Eigen::MatrixXd::Zero(6, 1);
  err.block<3, 1>(0, 0) = pos_err;
  err.block<3, 1>(3, 0) = ori_err_dash;

  return err;
}

bool EngineerKinematicsDynamics::calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                                                  int max_iter, double ik_err)
{
  bool ik_success = false;
  bool limit_success = false;

  //  calcForwardKinematics(0);

  std::vector<int> idx = findRoute(to);

  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);

    Eigen::MatrixXd curr_position = robotis_engineer_link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation = robotis_engineer_link_data_[to]->orientation_;

    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm() < ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = jacobian * jacobian.transpose();
    Eigen::MatrixXd jacobian_inverse = jacobian.transpose() * jacobian_trans.inverse();

    Eigen::MatrixXd delta_angle = jacobian_inverse * err;

    for (int id = 0; id < idx.size(); id++)
    {
      int joint_num = idx[id];
      robotis_engineer_link_data_[joint_num]->joint_angle_ += delta_angle.coeff(id);  //why coeff???
    }

    calcForwardKinematics(0);  //for what?
  }

  for (int id = 0; id < idx.size(); id++)
  {
    int joint_num = idx[id];

    if (robotis_engineer_link_data_[joint_num]->joint_angle_ >= robotis_engineer_link_data_[joint_num]->joint_limit_max_)
    {
      limit_success = false;
      break;
    }
    else if (robotis_engineer_link_data_[joint_num]->joint_angle_ <= robotis_engineer_link_data_[joint_num]->joint_limit_min_)
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool EngineerKinematicsDynamics::calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position,
                                                  Eigen::MatrixXd tar_orientation, int max_iter, double ik_err)
{
  bool ik_success = false;
  bool limit_success = false;

  //  calcForwardKinematics(0);

  std::vector<int> idx = findRoute(from, to);

  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);

    // ROS_INFO("size? %d", idx.size());
    ROS_INFO("test0");
    for (int k=0; k<6; k++)
      ROS_INFO("%f, %f, %f, %f, %f", 
      jacobian(k,1), jacobian(k,2), jacobian(k,3), jacobian(k,4), jacobian(k,5));

    Eigen::MatrixXd curr_position = robotis_engineer_link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation = robotis_engineer_link_data_[to]->orientation_;

    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    ROS_INFO("test1");
    for (int k=0; k<6; k++)
      ROS_INFO("%f", err(k));

    if (err.norm() < ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = jacobian * jacobian.transpose();
    Eigen::MatrixXd jacobian_inv = jacobian.transpose() * jacobian_trans.inverse();
    for (int k=0; k<5; k++)
      ROS_INFO("%f, %f, %f, %f, %f, %f", 
      jacobian_inv(k,1), jacobian_inv(k,2), jacobian_inv(k,3), 
      jacobian_inv(k,4), jacobian_inv(k,5), jacobian_inv(k,6));

    Eigen::MatrixXd delta_angle = jacobian_inv * err;

    ROS_INFO("test2");
    for (int k=0; k<6; k++)
      ROS_INFO("%f", delta_angle(k));


    for (int id = 0; id < idx.size(); id++)
    {
      int joint_num = idx[id];
      robotis_engineer_link_data_[joint_num]->joint_angle_ += delta_angle.coeff(id);
    }

    calcForwardKinematics(0);
    ROS_INFO("%d", iter);
    ROS_INFO("%f", err.norm());
  }

  for (int id = 0; id < idx.size(); id++)
  {
    int joint_num = idx[id];

    if (robotis_engineer_link_data_[joint_num]->joint_angle_ >= robotis_engineer_link_data_[joint_num]->joint_limit_max_)
    {
      limit_success = false;
      break;
    }
    else if (robotis_engineer_link_data_[joint_num]->joint_angle_ <= robotis_engineer_link_data_[joint_num]->joint_limit_min_)
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    ROS_INFO("%d", ik_success);
    ROS_INFO("%d", limit_success);
    return false;
}

bool EngineerKinematicsDynamics::calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                                                  int max_iter, double ik_err, Eigen::MatrixXd weight)
{
  bool ik_success = false;
  bool limit_success = false;

  //  calcForwardKinematics(0);   //why muted???

  std::vector<int> idx = findRoute(to);

  /* weight */
  Eigen::MatrixXd weight_matrix = Eigen::MatrixXd::Identity(idx.size(), idx.size());

  for (int ix = 0; ix < idx.size(); ix++)
    weight_matrix.coeffRef(ix, ix) = weight.coeff(idx[ix], 0);

  /* damping */
  Eigen::MatrixXd eval = Eigen::MatrixXd::Zero(6, 6);

  double p_damping = 1e-5;
  double R_damping = 1e-5;

  for (int ix = 0; ix < 3; ix++)
  {
    eval.coeffRef(ix, ix) = p_damping;
    eval.coeffRef(ix + 3, ix + 3) = R_damping;
  }

  /* ik */
  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);

    Eigen::MatrixXd curr_position = robotis_engineer_link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation = robotis_engineer_link_data_[to]->orientation_;

    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm() < ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = (jacobian * weight_matrix * jacobian.transpose() + eval);    //not good naming... jaco_trans...
    Eigen::MatrixXd jacobian_inv = weight_matrix * jacobian.transpose() * jacobian_trans.inverse();

    Eigen::MatrixXd delta_angle = jacobian_inv * err;

    for (int id = 0; id < idx.size(); id++)
    {
      int joint_id = idx[id];
      robotis_engineer_link_data_[joint_id]->joint_angle_ += delta_angle.coeff(id);
    }

    calcForwardKinematics(0);
  }

  /* check joint limit */
  for (int id = 0; id < idx.size(); id++)
  {
    int joint_num = idx[id];

    if (robotis_engineer_link_data_[joint_num]->joint_angle_ >= robotis_engineer_link_data_[joint_num]->joint_limit_max_)
    {
      limit_success = false;
      break;
    }
    else if (robotis_engineer_link_data_[joint_num]->joint_angle_ <= robotis_engineer_link_data_[joint_num]->joint_limit_min_)
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool EngineerKinematicsDynamics::calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position,
                                                  Eigen::MatrixXd tar_orientation, int max_iter, double ik_err,
                                                  Eigen::MatrixXd weight)
{
  bool ik_success = false;
  bool limit_success = false;

  //  calcForwardKinematics(0);

  std::vector<int> idx = findRoute(from, to);

  /* weight */
  Eigen::MatrixXd weight_matrix = Eigen::MatrixXd::Identity(idx.size(), idx.size());

  for (int ix = 0; ix < idx.size(); ix++)
    weight_matrix.coeffRef(ix, ix) = weight.coeff(idx[ix], 0);

  /* damping */
  Eigen::MatrixXd eval = Eigen::MatrixXd::Zero(6, 6);

  double p_damping = 1e-5;
  double R_damping = 1e-5;

  for (int ix = 0; ix < 3; ix++)
  {
    eval.coeffRef(ix, ix) = p_damping;
    eval.coeffRef(ix + 3, ix + 3) = R_damping;
  }

  /* ik */
  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);
    Eigen::MatrixXd curr_position = robotis_engineer_link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation = robotis_engineer_link_data_[to]->orientation_;
    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm() < ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = (jacobian * weight_matrix * jacobian.transpose() + eval);
    Eigen::MatrixXd jacobian_inv = weight_matrix * jacobian.transpose() * jacobian_trans.inverse();
    Eigen::MatrixXd delta_angle = jacobian_inv * err;

    for (int id = 0; id < idx.size(); id++)
    {
      int joint_id = idx[id];
      robotis_engineer_link_data_[joint_id]->joint_angle_ += delta_angle.coeff(id);
    }
    calcForwardKinematics(0);
  }

  /* check joint limit */
  for (int id = 0; id < idx.size(); id++)
  {
    int _joint_num = idx[id];
    if (robotis_engineer_link_data_[_joint_num]->joint_angle_ >= robotis_engineer_link_data_[_joint_num]->joint_limit_max_)
    {
      limit_success = false;
      break;
    }
    else if (robotis_engineer_link_data_[_joint_num]->joint_angle_ <= robotis_engineer_link_data_[_joint_num]->joint_limit_min_)
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool EngineerKinematicsDynamics::computeIK(double *out, double pos_x, double pos_y, double pos_z,
                              double ori_roll, double ori_pitch, double ori_yaw)
{
  // length
  double len_hipR2hipP = 0.024;         
  double len_hipP2kneeU = 0.060;     // hip pitch to knee upper
  double len_kneeU2kneeL = 0.0155;   // knee upper to knee lower
  double len_kneeL2ankleP = 0.060;   // knee lower to ankle pitch
  double len_ankleP2ankleR = 0.024;     
  double len_ankleR2ground = 0.0285; 
  // double height = (24.0+93.0+10.0+93.0+24.0+33.5) * 0.001; // should be current height.. modify here

  // angle
  double ang_hipR,      // Hip Roll Angle       
         ang_hipP,      // Hip Pitch Angle 
         ang_ankleP,    // Ankle Pitch Angle    
         ang_ankleR;    // Ankle Roll Angle      

  // Get Hip Roll
  Eigen::Vector3d pos_ankleR;     // Vector from ankle roll to hip roll
 
  Eigen::Matrix4d trans_sole;     // Transformation Matrix of Ankle Roll
  Eigen::Matrix4d trans_ankleR;   // Transformation Matrix of Ankle Roll

  trans_sole = robotis_framework::getTransformationXYZRPY(pos_x, pos_y, pos_z, ori_roll, 0, 0);

  Eigen::Vector3d vec_offset_ankleR2ground;  // Vector from Ankle Roll to Ground
  vec_offset_ankleR2ground << 0, 0, -len_ankleR2ground;

  pos_ankleR.coeffRef(0) = trans_sole.coeff(0, 3) + trans_sole.coeff(0, 2) * len_ankleR2ground;
  pos_ankleR.coeffRef(1) = trans_sole.coeff(1, 3) + trans_sole.coeff(1, 2) * len_ankleR2ground;
  pos_ankleR.coeffRef(2) = trans_sole.coeff(2, 3) + trans_sole.coeff(2, 2) * len_ankleR2ground;

  ang_hipR = atan2(pos_ankleR.coeffRef(1), -pos_ankleR.coeffRef(2));

  if (std::isnan(ang_hipR) == 1)
    return false;
  *(out) = ang_hipR;

  // Get Ankle Roll 
  ang_ankleR = -ang_hipR + ori_roll;

  if (std::isnan(ang_ankleR) == 1)   
    return false;
  *(out + 3) = ang_ankleR;

  // Get Ankle Pitch
  Eigen::Vector3d vec_hipP2ankleP;          // Vector from ankle roll to hip roll
  Eigen::Matrix3d ori_hipR;                 // Orientation of Ankle Pitch        
  ori_hipR = robotis_framework::convertRPYToRotation(ang_hipR, 0, 0);

  Eigen::Vector3d vec_offset_hipR2hipP;     // Vector from Ankle Roll to Ground    
  Eigen::Vector3d vec_offset_ankleP2ankleR; // Vector from Ankle Roll to Ground    
  vec_offset_hipR2hipP << 0, 0, -len_hipR2hipP;
  vec_offset_ankleP2ankleR << 0, 0, -len_ankleP2ankleR;

  vec_hipP2ankleP = pos_ankleR 
    - ori_hipR * vec_offset_hipR2hipP
    - ori_hipR * vec_offset_ankleP2ankleR;

  double len_hipP2ankleP; 
  double alpha;            // angle between vec_hipP2ankleP and vec_kneeL2ankleP
  double cos_alpha;       
  double beta;             // angle between vec_hipP2ankleP and vec_hipP2kneeU

  len_hipP2ankleP = vec_hipP2ankleP.norm();

  cos_alpha = ((len_hipP2ankleP-len_kneeU2kneeL) * (len_hipP2ankleP-len_kneeU2kneeL)
    - len_hipP2kneeU*len_hipP2kneeU 
    + len_kneeL2ankleP*len_kneeL2ankleP)
    / (2 * len_kneeL2ankleP * (len_hipP2ankleP-len_kneeU2kneeL));
  
  // ROS_INFO("cos_alpha: %f", cos_alpha);
  if (std::fabs(cos_alpha - 1) < 0.00001) // rewrite here...
    cos_alpha = 1; 

  alpha = acos(cos_alpha);

  ang_ankleP = -atan2(vec_hipP2ankleP.coeffRef(0), 
    sqrt(vec_hipP2ankleP.coeffRef(1)*vec_hipP2ankleP.coeffRef(1) 
    + vec_hipP2ankleP.coeffRef(2)*vec_hipP2ankleP.coeffRef(2))) 
    - acos(cos_alpha);

  if (std::isnan(ang_ankleP) == 1)   
    return false;
  *(out + 2) = ang_ankleP;

  // Get Hip Pitch
  beta = asin(len_kneeL2ankleP / len_hipP2kneeU * sin(alpha));
  ang_hipP = ang_ankleP + alpha - beta;

  if (std::isnan(ang_hipP) == 1) 
    return false;
  *(out + 1) = ang_hipP;

  return true;
}

bool EngineerKinematicsDynamics::calcInverseKinematicsForRightLeg(double *out, double x, double y, double z, 
                                                            double roll, double pitch, double yaw)
{
  if (computeIK(out, x, y, z, roll, pitch, yaw) == true)
  {
    out[0] *= getJointDirection(5);
    out[1] *= getJointDirection(6);
    out[2] *= getJointDirection(9);
    out[3] *= getJointDirection(10);

    return true;
  }
  else
    return false;
}

bool EngineerKinematicsDynamics::calcInverseKinematicsForLeftLeg(double *out, double x, double y, double z, 
                                                            double roll, double pitch, double yaw)
{
  if (computeIK(out, x, y, z, roll, pitch, yaw) == true)
  {
    out[0] *= getJointDirection(7);
    out[1] *= getJointDirection(8);
    out[2] *= getJointDirection(11);
    out[3] *= getJointDirection(12);

    return true;
  }
  else
    return false;
}

LinkData *EngineerKinematicsDynamics::getLinkData(const std::string link_name)
{
  for (int ix = 0; ix <= ALL_JOINT_ID; ix++)
  {
    if (robotis_engineer_link_data_[ix]->name_ == link_name)
    {
      return robotis_engineer_link_data_[ix];
    }
  }

  return NULL;
}

LinkData *EngineerKinematicsDynamics::getLinkData(const int link_id)
{
  if (robotis_engineer_link_data_[link_id] != NULL)
  {
    return robotis_engineer_link_data_[link_id];
  }

  return NULL;
}

Eigen::MatrixXd EngineerKinematicsDynamics::getJointAxis(const std::string link_name)
{
  Eigen::MatrixXd joint_axis;

  LinkData *link_data = getLinkData(link_name);

  if (link_data != NULL)
  {
    joint_axis = link_data->joint_axis_;
  }

  return joint_axis;
}

double EngineerKinematicsDynamics::getJointDirection(const std::string link_name)
{
  double joint_direction = 0.0;
  LinkData *link_data = getLinkData(link_name);

  if (link_data != NULL)
  {
    joint_direction = link_data->joint_axis_.coeff(0, 0) + link_data->joint_axis_.coeff(1, 0)
        + link_data->joint_axis_.coeff(2, 0);
  }

  return joint_direction;
}

double EngineerKinematicsDynamics::getJointDirection(const int link_id)
{
  double joint_direction = 0.0;
  LinkData *link_data = getLinkData(link_id);

  if (link_data != NULL)
  {
    joint_direction = link_data->joint_axis_.coeff(0, 0) + link_data->joint_axis_.coeff(1, 0)
        + link_data->joint_axis_.coeff(2, 0);
  }

  return joint_direction;
}

Eigen::MatrixXd EngineerKinematicsDynamics::calcPreviewParam(double preview_time, double control_cycle,
                                                        double lipm_height,
                                                        Eigen::MatrixXd K, Eigen::MatrixXd P)
{
  double t = control_cycle;
  double preview_size_ = round(preview_time/control_cycle) + 1;

  Eigen::MatrixXd A_;
  A_.resize(3,3);
  A_ << 1,  t,  t*t/2.0,
        0,  1,  t,
        0,  0,  1;

  Eigen::MatrixXd b_;
  b_.resize(3,1);
  b_ << t*t*t/6.0,
        t*t/2.0,
        t;

  Eigen::MatrixXd c_;
  c_.resize(1,3);
  c_ << 1, 0, -lipm_height/9.81;

  Eigen::MatrixXd tempA = Eigen::MatrixXd::Zero(4,4);
  Eigen::MatrixXd tempb = Eigen::MatrixXd::Zero(4,1);
  Eigen::MatrixXd tempc = Eigen::MatrixXd::Zero(1,4);

  tempA.coeffRef(0,0) = 1;
  tempA.block<1,3>(0,1) = c_*A_;
  tempA.block<3,3>(1,1) = A_;

  tempb.coeffRef(0,0) = (c_*b_).coeff(0,0);
  tempb.block<3,1>(1,0) = b_;

  tempc.coeffRef(0,0) = 1;

  double R = 1e-6;
  double Q_e = 1;
  double Q_x = 0;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4,4);
  Q.coeffRef(0,0) = Q_e;
  Q.coeffRef(1,1) = Q_e;
  Q.coeffRef(2,2) = Q_e;
  Q.coeffRef(3,3) = Q_x;

  Eigen::MatrixXd f_;
  f_.resize(1, preview_size_);

  Eigen::MatrixXd mat_R = Eigen::MatrixXd::Zero(1,1);
  mat_R.coeffRef(0,0) = R;

  Eigen::MatrixXd tempCoeff1 = mat_R + ((tempb.transpose() * P) * tempb);
  Eigen::MatrixXd tempCoeff1_inv = tempCoeff1.inverse();
  Eigen::MatrixXd tempCoeff2 = tempb.transpose();
  Eigen::MatrixXd tempCoeff3 = Eigen::MatrixXd::Identity(4,4);
  Eigen::MatrixXd tempCoeff4 = P*tempc.transpose();

  f_.block<1,1>(0,0) = ((tempCoeff1_inv*tempCoeff2)* tempCoeff3) * tempCoeff4;

  for(int i = 1; i < preview_size_; i++)
  {
    tempCoeff3 = tempCoeff3*((tempA - tempb*K).transpose());
    f_.block<1,1>(0,i) = ((tempCoeff1_inv*tempCoeff2)* tempCoeff3) * tempCoeff4;
  }

  return f_;
}

}
