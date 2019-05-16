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

/* Author: SCH, Jay Song, Kayman */

#include <iostream>
#include "pre_kinematics_dynamics/pre_kinematics_dynamics.h"

namespace robotis_op
{

OP3KinematicsDynamics::OP3KinematicsDynamics()
{
}
OP3KinematicsDynamics::~OP3KinematicsDynamics()
{
}

OP3KinematicsDynamics::OP3KinematicsDynamics(TreeSelect tree)
{
  for (int id = 0; id <= ALL_JOINT_ID; id++)
    op3_link_data_[id] = new LinkData();

  if (tree == WholeBody)
  {
    op3_link_data_[0]->name_ = "base";
    op3_link_data_[0]->parent_ = -1;
    op3_link_data_[0]->sibling_ = -1;
    op3_link_data_[0]->child_ = 1;
    op3_link_data_[0]->mass_ = 0.0;
    op3_link_data_[0]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[0]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[0]->joint_limit_max_ = 100.0;
    op3_link_data_[0]->joint_limit_min_ = -100.0;


    /*----- right arm -----*/

    // right arm shoulder pitch
    op3_link_data_[1]->name_ = "r_shoulder_pitch";
    op3_link_data_[1]->parent_ = 0;
    op3_link_data_[1]->sibling_ = 3;
    op3_link_data_[1]->child_ = 2;
    op3_link_data_[1]->mass_ = 0.0;
    op3_link_data_[1]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0); // (-0.0714*cos(M_PI/12), 0.0714*sin(M_PI/12), 0)
    op3_link_data_[1]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.9659, -0.2588);        // (0, cos(M_PI/12), -sin(M_PI/12))
    op3_link_data_[1]->joint_limit_max_ = 0.5 * 180;
    op3_link_data_[1]->joint_limit_min_ = -0.5 * 180;

    // right arm shoulder roll
    op3_link_data_[2]->name_ = "r_shoulder_roll";
    op3_link_data_[2]->parent_ = 1;
    op3_link_data_[2]->sibling_ = -1;
    op3_link_data_[2]->child_ = 13;
    op3_link_data_[2]->mass_ = 0.0;
    op3_link_data_[2]->relative_position_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    op3_link_data_[2]->joint_axis_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    op3_link_data_[2]->joint_limit_max_ = 0.3 * 180;
    op3_link_data_[2]->joint_limit_min_ = -0.5 * 180;

    // right arm end effector
    op3_link_data_[13]->name_ = "r_endEffector";
    op3_link_data_[13]->parent_ = 2;
    op3_link_data_[13]->sibling_ = -1;
    op3_link_data_[13]->child_ = -1;
    op3_link_data_[13]->mass_ = 0.0;
    op3_link_data_[13]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[13]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[13]->joint_limit_max_ = 0;
    op3_link_data_[13]->joint_limit_min_ = 0;
  

    /*----- left arm -----*/

    // left arm shoulder pitch
    op3_link_data_[3]->name_ = "l_shoulder_pitch";
    op3_link_data_[3]->parent_ = 0;
    op3_link_data_[3]->sibling_ = 5;
    op3_link_data_[3]->child_ = 4;
    op3_link_data_[3]->mass_ = 0.0;
    op3_link_data_[3]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0); // (0, 0.0714*cos(M_PI/12), 0.0714*sin(M_PI/12))
    op3_link_data_[3]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);        // (0, cos(M_PI/12), sin(M_PI/12))
    op3_link_data_[3]->joint_limit_max_ = 100 * 180;
    op3_link_data_[3]->joint_limit_min_ = -100 * 180;

    // left arm shoulder roll
    op3_link_data_[4]->name_ = "l_shoulder_roll";
    op3_link_data_[4]->parent_ = 3;
    op3_link_data_[4]->sibling_ = -1;
    op3_link_data_[4]->child_ = 7;
    op3_link_data_[4]->mass_ = 0.0;
    op3_link_data_[4]->relative_position_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    op3_link_data_[4]->joint_axis_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    op3_link_data_[4]->joint_limit_max_ = 100 * 180;
    op3_link_data_[4]->joint_limit_min_ = -100 * 180;

    // left arm end effector
    op3_link_data_[14]->name_ = "l_endEffector";
    op3_link_data_[14]->parent_ = 4;
    op3_link_data_[14]->sibling_ = -1;
    op3_link_data_[14]->child_ = -1;
    op3_link_data_[14]->mass_ = 0.0;
    op3_link_data_[14]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[14]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[14]->joint_limit_max_ = 100 * 180;
    op3_link_data_[14]->joint_limit_min_ = -100 * 180;


    /*----- right leg -----*/

    // right hip roll
    op3_link_data_[5]->name_ = "r_hip_roll";
    op3_link_data_[5]->parent_ = 0;
    op3_link_data_[5]->sibling_ = 7;
    op3_link_data_[5]->child_ = 6;
    op3_link_data_[5]->mass_ = 0.0;
    op3_link_data_[5]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, -0.0575, 0.0);
    op3_link_data_[5]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    op3_link_data_[5]->joint_limit_max_ = 0.5 * 180;
    op3_link_data_[5]->joint_limit_min_ = -0.5 * 180;

    // right hip pitch
    op3_link_data_[6]->name_ = "r_hip_pitch";
    op3_link_data_[6]->parent_ = 5;
    op3_link_data_[6]->sibling_ = 9;
    op3_link_data_[6]->child_ = -1;
    op3_link_data_[6]->mass_ = 0.194;
    op3_link_data_[6]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, -0.0575, 0.0);
    op3_link_data_[6]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    op3_link_data_[6]->joint_limit_max_ = 0.5 * 180;
    op3_link_data_[6]->joint_limit_min_ = -0.5 * 180;

    // right ankle pitch
    op3_link_data_[9]->name_ = "r_ankle_roll";
    op3_link_data_[9]->parent_ = 6;
    op3_link_data_[9]->sibling_ = -1;
    op3_link_data_[9]->child_ = 10;
    op3_link_data_[9]->mass_ = 0.0;
    op3_link_data_[9]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, -0.0575, 0.0);
    op3_link_data_[9]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    op3_link_data_[9]->joint_limit_max_ = 0.5 * 180;
    op3_link_data_[9]->joint_limit_min_ = -0.5 * 180;

    // right ankle roll
    op3_link_data_[10]->name_ = "r_ankle_pitch";
    op3_link_data_[10]->parent_ = 9;
    op3_link_data_[10]->sibling_ = -1;
    op3_link_data_[10]->child_ = 15;
    op3_link_data_[10]->mass_ = 0.0;
    op3_link_data_[10]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, -0.0575, 0.0);
    op3_link_data_[10]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    op3_link_data_[10]->joint_limit_max_ = 0.5 * 180;
    op3_link_data_[10]->joint_limit_min_ = -0.5 * 180;

    // right sole
    op3_link_data_[15]->name_ = "r_sole";
    op3_link_data_[15]->parent_ = 10;
    op3_link_data_[15]->sibling_ = -1;
    op3_link_data_[15]->child_ = -1;
    op3_link_data_[15]->mass_ = 0.0;
    op3_link_data_[15]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, -0.0575, 0.0);
    op3_link_data_[15]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    op3_link_data_[15]->joint_limit_max_ = 0.5 * 180;
    op3_link_data_[15]->joint_limit_min_ = -0.5 * 180;

    /*----- left leg -----*/

    // left hip roll
    op3_link_data_[7]->name_ = "l_hip_roll";
    op3_link_data_[7]->parent_ = 0;
    op3_link_data_[7]->sibling_ = -1;
    op3_link_data_[7]->child_ = 8;
    op3_link_data_[7]->mass_ = 0.0;
    op3_link_data_[7]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, -0.0575, 0.0);
    op3_link_data_[7]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    op3_link_data_[7]->joint_limit_max_ = 0.5 * 180;
    op3_link_data_[7]->joint_limit_min_ = -0.5 * 180;

    // left hip pitch
    op3_link_data_[8]->name_ = "l_hip_pitch";
    op3_link_data_[8]->parent_ = 7;
    op3_link_data_[8]->sibling_ = -1;
    op3_link_data_[8]->child_ = 11;
    op3_link_data_[8]->mass_ = 0.194;
    op3_link_data_[8]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, -0.0575, 0.0);
    op3_link_data_[8]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    op3_link_data_[8]->joint_limit_max_ = 0.5 * 180;
    op3_link_data_[8]->joint_limit_min_ = -0.5 * 180;

    // left ankle pitch
    op3_link_data_[11]->name_ = "l_ankle_roll";
    op3_link_data_[11]->parent_ = 8;
    op3_link_data_[11]->sibling_ = -1;
    op3_link_data_[11]->child_ = 12;
    op3_link_data_[11]->mass_ = 0.0;
    op3_link_data_[11]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, -0.0575, 0.0);
    op3_link_data_[11]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    op3_link_data_[11]->joint_limit_max_ = 0.5 * 180;
    op3_link_data_[11]->joint_limit_min_ = -0.5 * 180;

    // left ankle roll
    op3_link_data_[12]->name_ = "l_ankle_pitch";
    op3_link_data_[12]->parent_ = 11;
    op3_link_data_[12]->sibling_ = -1;
    op3_link_data_[12]->child_ = 14;
    op3_link_data_[12]->mass_ = 0.0;
    op3_link_data_[12]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, -0.0575, 0.0);
    op3_link_data_[12]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    op3_link_data_[12]->joint_limit_max_ = 0.5 * 180;
    op3_link_data_[12]->joint_limit_min_ = -0.5 * 180;

    // left sole
    op3_link_data_[16]->name_ = "l_sole";
    op3_link_data_[16]->parent_ = 12;
    op3_link_data_[16]->sibling_ = -1;
    op3_link_data_[16]->child_ = -1;
    op3_link_data_[16]->mass_ = 0.0;
    op3_link_data_[16]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, -0.0575, 0.0);
    op3_link_data_[16]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[16]->joint_limit_max_ = 0.0;
    op3_link_data_[16]->joint_limit_min_ = 0.0;
  }

  thigh_length_m_ = std::fabs(op3_link_data_[ID_R_LEG_START + 2 * 3]->relative_position_.coeff(2, 0));
  calf_length_m_ = std::fabs(op3_link_data_[ID_R_LEG_START + 2 * 4]->relative_position_.coeff(2, 0));
  ankle_length_m_ = std::fabs(op3_link_data_[ID_R_LEG_END]->relative_position_.coeff(2, 0));
  leg_side_offset_m_ = 2.0 * (std::fabs(op3_link_data_[ID_R_LEG_START]->relative_position_.coeff(1, 0)));
}
// didnt consider to = 0
std::vector<int> OP3KinematicsDynamics::findRoute(int to)  
{
  int id = op3_link_data_[to]->parent_;

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
std::vector<int> OP3KinematicsDynamics::findRoute(int from, int to) 
{
  int id = op3_link_data_[to]->parent_;

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

double OP3KinematicsDynamics::calcTotalMass(int joint_id)
{
  double mass;

  if (joint_id == -1)
    mass = 0.0;
  else
    mass = op3_link_data_[joint_id]->mass_ 
        + calcTotalMass(op3_link_data_[joint_id]->sibling_)
        + calcTotalMass(op3_link_data_[joint_id]->child_);

  return mass;
}

Eigen::MatrixXd OP3KinematicsDynamics::calcMC(int joint_id)
{
  Eigen::MatrixXd mc(3, 1);

  if (joint_id == -1)
    mc = Eigen::MatrixXd::Zero(3, 1);
  else
  {
    mc = op3_link_data_[joint_id]->mass_
      * (op3_link_data_[joint_id]->orientation_ 
        * op3_link_data_[joint_id]->center_of_mass_
        + op3_link_data_[joint_id]->position_);
    mc = mc 
      + calcMC(op3_link_data_[joint_id]->sibling_) 
      + calcMC(op3_link_data_[joint_id]->child_);
  }

  return mc;
}
// better write MC and M, using small letters cause misunderstanding.
Eigen::MatrixXd OP3KinematicsDynamics::calcCOM(Eigen::MatrixXd mc)
{
  double mass;                      
  Eigen::MatrixXd COM(3, 1);

  mass = calcTotalMass(0);
  COM = mc / mass;

  return COM;
}

void OP3KinematicsDynamics::calcForwardKinematics(int joint_id)
{
  if (joint_id == -1)
    return;

  if (joint_id == 0)
  {
    op3_link_data_[0]->position_ = Eigen::MatrixXd::Zero(3, 1);
    op3_link_data_[0]->orientation_ = robotis_framework::calcRodrigues(
        robotis_framework::calcHatto(op3_link_data_[0]->joint_axis_),
                                    op3_link_data_[0]->joint_angle_);
  }

  if (joint_id != 0)
  {
    int parent = op3_link_data_[joint_id]->parent_;

    op3_link_data_[joint_id]->position_ = op3_link_data_[parent]->orientation_
        * op3_link_data_[joint_id]->relative_position_ + op3_link_data_[parent]->position_;
    op3_link_data_[joint_id]->orientation_ = op3_link_data_[parent]->orientation_
        * robotis_framework::calcRodrigues(robotis_framework::calcHatto(op3_link_data_[joint_id]->joint_axis_),
                                           op3_link_data_[joint_id]->joint_angle_);

    op3_link_data_[joint_id]->transformation_.block<3, 1>(0, 3) = op3_link_data_[joint_id]->position_;
    op3_link_data_[joint_id]->transformation_.block<3, 3>(0, 0) = op3_link_data_[joint_id]->orientation_;
  }

  calcForwardKinematics(op3_link_data_[joint_id]->sibling_);
  calcForwardKinematics(op3_link_data_[joint_id]->child_);
}

Eigen::MatrixXd OP3KinematicsDynamics::calcJacobian(std::vector<int> idx)
{
  int idx_size = idx.size();
  int end = idx_size - 1;

  Eigen::MatrixXd tar_position = op3_link_data_[idx[end]]->position_;
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, idx_size);

  for (int id = 0; id < idx_size; id++)
  {
    int curr_id = idx[id];

    Eigen::MatrixXd tar_orientation = op3_link_data_[curr_id]->orientation_ * op3_link_data_[curr_id]->joint_axis_;

    jacobian.block(0, id, 3, 1) = robotis_framework::calcCross(tar_orientation,
                                                               tar_position - op3_link_data_[curr_id]->position_);
    jacobian.block(3, id, 3, 1) = tar_orientation;    // difference from the other one? about block function/?
  }

  return jacobian;
}

Eigen::MatrixXd OP3KinematicsDynamics::calcJacobianCOM(std::vector<int> idx)
{
  int idx_size = idx.size();
  int end = idx_size - 1;

  Eigen::MatrixXd tar_position = op3_link_data_[idx[end]]->position_;
  Eigen::MatrixXd jacobian_com = Eigen::MatrixXd::Zero(6, idx_size);

  for (int id = 0; id < idx_size; id++)
  {
    int curr_id = idx[id];
    double mass = calcTotalMass(curr_id);

    Eigen::MatrixXd og = calcMC(curr_id) / mass - op3_link_data_[curr_id]->position_;  // COM - position   )   og stands for ??
    Eigen::MatrixXd tar_orientation = op3_link_data_[curr_id]->orientation_ * op3_link_data_[curr_id]->joint_axis_;

    jacobian_com.block(0, id, 3, 1) = robotis_framework::calcCross(tar_orientation, og); // target position is replaced with COM
    jacobian_com.block(3, id, 3, 1) = tar_orientation;
  }

  return jacobian_com;
}

Eigen::MatrixXd OP3KinematicsDynamics::calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position,
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

bool OP3KinematicsDynamics::calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                                                  int max_iter, double ik_err)
{
  bool ik_success = false;
  bool limit_success = false;

  //  calcForwardKinematics(0);

  std::vector<int> idx = findRoute(to);

  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);

    Eigen::MatrixXd curr_position = op3_link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation = op3_link_data_[to]->orientation_;

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
      op3_link_data_[joint_num]->joint_angle_ += delta_angle.coeff(id);  //why coeff???
    }

    calcForwardKinematics(0);  //for what?
  }

  for (int id = 0; id < idx.size(); id++)
  {
    int joint_num = idx[id];

    if (op3_link_data_[joint_num]->joint_angle_ >= op3_link_data_[joint_num]->joint_limit_max_)
    {
      limit_success = false;
      break;
    }
    else if (op3_link_data_[joint_num]->joint_angle_ <= op3_link_data_[joint_num]->joint_limit_min_)
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

bool OP3KinematicsDynamics::calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position,
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

    Eigen::MatrixXd curr_position = op3_link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation = op3_link_data_[to]->orientation_;

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
      op3_link_data_[joint_num]->joint_angle_ += delta_angle.coeff(id);
    }

    calcForwardKinematics(0);
    ROS_INFO("%d", iter);
    ROS_INFO("%f", err.norm());
  }

  for (int id = 0; id < idx.size(); id++)
  {
    int joint_num = idx[id];

    if (op3_link_data_[joint_num]->joint_angle_ >= op3_link_data_[joint_num]->joint_limit_max_)
    {
      limit_success = false;
      break;
    }
    else if (op3_link_data_[joint_num]->joint_angle_ <= op3_link_data_[joint_num]->joint_limit_min_)
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

bool OP3KinematicsDynamics::calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
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

    Eigen::MatrixXd curr_position = op3_link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation = op3_link_data_[to]->orientation_;

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
      op3_link_data_[joint_id]->joint_angle_ += delta_angle.coeff(id);
    }

    calcForwardKinematics(0);
  }

  /* check joint limit */
  for (int id = 0; id < idx.size(); id++)
  {
    int joint_num = idx[id];

    if (op3_link_data_[joint_num]->joint_angle_ >= op3_link_data_[joint_num]->joint_limit_max_)
    {
      limit_success = false;
      break;
    }
    else if (op3_link_data_[joint_num]->joint_angle_ <= op3_link_data_[joint_num]->joint_limit_min_)
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

bool OP3KinematicsDynamics::calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position,
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
    Eigen::MatrixXd curr_position = op3_link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation = op3_link_data_[to]->orientation_;
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
      op3_link_data_[joint_id]->joint_angle_ += delta_angle.coeff(id);
    }
    calcForwardKinematics(0);
  }

  /* check joint limit */
  for (int id = 0; id < idx.size(); id++)
  {
    int _joint_num = idx[id];
    if (op3_link_data_[_joint_num]->joint_angle_ >= op3_link_data_[_joint_num]->joint_limit_max_)
    {
      limit_success = false;
      break;
    }
    else if (op3_link_data_[_joint_num]->joint_angle_ <= op3_link_data_[_joint_num]->joint_limit_min_)
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

bool OP3KinematicsDynamics::calcInverseKinematicsForLeg(double *out, double x, double y, double z, 
                                                        double roll, double pitch, double yaw)
{
  Eigen::Matrix4d trans_ad, trans_da, trans_cd, trans_dc, trans_ac;  //transpose? transformation?? why just trans???
  Eigen::Vector3d vec;

  bool invertible;
  double rac, arc_cos, arc_tan, k, l, m, n, s, c, theta;
  double thigh_length = thigh_length_m_;
  double calf_length = calf_length_m_;
  double ankle_length = ankle_length_m_;

  trans_ad = robotis_framework::getTransformationXYZRPY(x, y, z, roll, pitch, yaw);

  vec.coeffRef(0) = trans_ad.coeff(0, 3) + trans_ad.coeff(0, 2) * ankle_length;
  vec.coeffRef(1) = trans_ad.coeff(1, 3) + trans_ad.coeff(1, 2) * ankle_length;
  vec.coeffRef(2) = trans_ad.coeff(2, 3) + trans_ad.coeff(2, 2) * ankle_length;

  // Get Knee
  rac = vec.norm();
  arc_cos = acos(
      (rac * rac - thigh_length * thigh_length - calf_length * calf_length) / (2.0 * thigh_length * calf_length));
  if (std::isnan(arc_cos) == 1)
    return false;
  *(out + 3) = arc_cos;

  // Get Ankle Roll
  trans_ad.computeInverseWithCheck(trans_da, invertible);
  if (invertible == false)
    return false;

  k = sqrt(trans_da.coeff(1, 3) * trans_da.coeff(1, 3) + trans_da.coeff(2, 3) * trans_da.coeff(2, 3));
  l = sqrt(
      trans_da.coeff(1, 3) * trans_da.coeff(1, 3)
          + (trans_da.coeff(2, 3) - ankle_length) * (trans_da.coeff(2, 3) - ankle_length));
  m = (k * k - l * l - ankle_length * ankle_length) / (2.0 * l * ankle_length);

  if (m > 1.0)
    m = 1.0;
  else if (m < -1.0)
    m = -1.0;
  arc_cos = acos(m);

  if (std::isnan(arc_cos) == 1)
    return false;

  if (trans_da.coeff(1, 3) < 0.0)
    *(out + 5) = -arc_cos;
  else
    *(out + 5) = arc_cos;

  // Get Hip Yaw
  trans_cd = robotis_framework::getTransformationXYZRPY(0, 0, -ankle_length, *(out + 5), 0, 0);
  trans_cd.computeInverseWithCheck(trans_dc, invertible);
  if (invertible == false)
    return false;

  trans_ac = trans_ad * trans_dc;
  arc_tan = atan2(-trans_ac.coeff(0, 1), trans_ac.coeff(1, 1));
  if (std::isinf(arc_tan) != 0)
    return false;
  *(out) = arc_tan;

  // Get Hip Roll
  arc_tan = atan2(trans_ac.coeff(2, 1), -trans_ac.coeff(0, 1) * sin(*(out)) + trans_ac.coeff(1, 1) * cos(*(out)));
  if (std::isinf(arc_tan) != 0)
    return false;
  *(out + 1) = arc_tan;

  // Get Hip Pitch and Ankle Pitch
  arc_tan = atan2(trans_ac.coeff(0, 2) * cos(*(out)) + trans_ac.coeff(1, 2) * sin(*(out)),
                  trans_ac.coeff(0, 0) * cos(*(out)) + trans_ac.coeff(1, 0) * sin(*(out)));
  if (std::isinf(arc_tan) == 1)
    return false;
  theta = arc_tan;
  k = sin(*(out + 3)) * calf_length;
  l = -thigh_length - cos(*(out + 3)) * calf_length;
  m = cos(*(out)) * vec.coeff(0) + sin(*(out)) * vec.coeff(1);
  n = cos(*(out + 1)) * vec.coeff(2) + sin(*(out)) * sin(*(out + 1)) * vec.coeff(0)
      - cos(*(out)) * sin(*(out + 1)) * vec.coeff(1);
  s = (k * n + l * m) / (k * k + l * l);
  c = (n - k * s) / l;
  arc_tan = atan2(s, c);
  if (std::isinf(arc_tan) == 1)
    return false;
  *(out + 2) = arc_tan;
  *(out + 4) = theta - *(out + 3) - *(out + 2);

  return true;
}
// can integrate the below two using if left or right 
bool OP3KinematicsDynamics::calcInverseKinematicsForRightLeg(double *out, double x, double y, double z, 
                                                            double roll, double pitch, double yaw)
{
  if (calcInverseKinematicsForLeg(out, x, y, z, roll, pitch, yaw) == true)
  {
    for(int ix = 0 ; ix < 6; ix++)
      out[ix] *= getJointDirection(ID_R_LEG_START + 2 * ix);

    return true;
  }
  else
    return false;
}

bool OP3KinematicsDynamics::calcInverseKinematicsForLeftLeg(double *out, double x, double y, double z, 
                                                            double roll, double pitch, double yaw)
{
  if (calcInverseKinematicsForLeg(out, x, y, z, roll, pitch, yaw) == true)
  {
    for(int ix = 0 ; ix < 6; ix++)
      out[ix] *= getJointDirection(ID_L_LEG_START + 2 * ix);

    return true;
  }
  else
    return false;
}

LinkData *OP3KinematicsDynamics::getLinkData(const std::string link_name)
{
  for (int ix = 0; ix <= ALL_JOINT_ID; ix++)
  {
    if (op3_link_data_[ix]->name_ == link_name)
    {
      return op3_link_data_[ix];
    }
  }

  return NULL;
}

LinkData *OP3KinematicsDynamics::getLinkData(const int link_id)
{
  if (op3_link_data_[link_id] != NULL)
  {
    return op3_link_data_[link_id];
  }

  return NULL;
}

Eigen::MatrixXd OP3KinematicsDynamics::getJointAxis(const std::string link_name)
{
  Eigen::MatrixXd joint_axis;

  LinkData *link_data = getLinkData(link_name);

  if (link_data != NULL)
  {
    joint_axis = link_data->joint_axis_;
  }

  return joint_axis;
}

double OP3KinematicsDynamics::getJointDirection(const std::string link_name)
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

double OP3KinematicsDynamics::getJointDirection(const int link_id)
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

Eigen::MatrixXd OP3KinematicsDynamics::calcPreviewParam(double preview_time, double control_cycle,
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
