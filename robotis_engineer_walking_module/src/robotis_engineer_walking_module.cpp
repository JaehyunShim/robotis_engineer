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

/* Author: Kayman */

#include "robotis_engineer_walking_module/robotis_engineer_walking_module.h"

namespace robotis_engineer
{

WalkingModule::WalkingModule()
    : control_cycle_msec_(8),
      DEBUG(false)
{
  enable_ = false;
  module_name_ = "walking_module";
  control_mode_ = robotis_framework::PositionControl;

  init_pose_count_ = 0;
  walking_state_ = WalkingReady;
  previous_x_move_amplitude_ = 0.0;

  robotis_engineer_kd_ = new EngineerKinematicsDynamics(WholeBody);   

  // result
  result_["r_shoulder_pitch"] = new robotis_framework::DynamixelState();
  result_["r_shoulder_roll"] = new robotis_framework::DynamixelState();
  result_["l_shoulder_pitch"] = new robotis_framework::DynamixelState();
  result_["l_shoulder_roll"] = new robotis_framework::DynamixelState();

  result_["r_hip_roll"] = new robotis_framework::DynamixelState();
  result_["r_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["r_ankle_pitch"] = new robotis_framework::DynamixelState();
  result_["r_ankle_roll"] = new robotis_framework::DynamixelState();

  result_["l_hip_roll"] = new robotis_framework::DynamixelState();
  result_["l_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["l_ankle_pitch"] = new robotis_framework::DynamixelState();
  result_["l_ankle_roll"] = new robotis_framework::DynamixelState();


  // joint table
  joint_table_["r_shoulder_pitch"] = 0;
  joint_table_["r_shoulder_roll"] = 1;
  joint_table_["l_shoulder_pitch"] = 2;
  joint_table_["l_shoulder_roll"] = 3;

  joint_table_["r_hip_roll"] = 4;
  joint_table_["r_hip_pitch"] = 5;
  joint_table_["r_ankle_pitch"] = 6;
  joint_table_["r_ankle_roll"] = 7;

  joint_table_["l_hip_roll"] = 8;
  joint_table_["l_hip_pitch"] = 9;
  joint_table_["l_ankle_pitch"] = 10;
  joint_table_["l_ankle_roll"] = 11;

  init_position_ = Eigen::MatrixXd::Zero(1, result_.size());   // Init position  
  goal_position_ = Eigen::MatrixXd::Zero(1, result_.size());   // Goal position
  target_position_ = Eigen::MatrixXd::Zero(1, result_.size()); // Position used to linearlize the motion to the goal position?
}

WalkingModule::~WalkingModule()
{
  queue_thread_.join();
}

void WalkingModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  queue_thread_ = boost::thread(boost::bind(&WalkingModule::queueThread, this));
  control_cycle_msec_ = control_cycle_msec;

  // m, s, rad
  // init pose    Is this part really needed???
  walking_param_.init_x_offset = 0.0;                     // init x
  walking_param_.init_y_offset = 0.0;                     // init y
  walking_param_.init_z_offset = 0.0;                     // init z
  walking_param_.init_roll_offset = 0.0 * DEGREE2RADIAN;  // init roll
  walking_param_.init_pitch_offset = 0.0 * DEGREE2RADIAN; // init pitch
  walking_param_.init_yaw_offset = 0.0 * DEGREE2RADIAN;   // init yaw
  walking_param_.hip_pitch_offset = 0.0 * DEGREE2RADIAN;  // init hip pitch
  // time
  walking_param_.period_time = 10000 * 0.001;  // leg motion duration
  walking_param_.dsp_ratio = 0.30;             // double stance phase
  walking_param_.step_fb_ratio = 0.0;          // Used for the motion of the torso in the anteroposterior direction during gait
  // walking
  walking_param_.x_move_amplitude = 0.0;       // leg??
  walking_param_.y_move_amplitude = 0.0;       // leg??
  walking_param_.z_move_amplitude = 0.020;     // Height of the foot raising
  walking_param_.angle_move_amplitude = 0.0;   // Rotationtal motion
  // balance
  walking_param_.balance_enable = false;
  walking_param_.balance_hip_roll_gain = 0.0;
  walking_param_.balance_knee_gain = 0.0;
  walking_param_.balance_ankle_roll_gain = 0.0;
  walking_param_.balance_ankle_pitch_gain = 0.0;
  walking_param_.y_swap_amplitude = 0.0;   // Used for the motion of the torso in the transverse direction during gait 
  walking_param_.z_swap_amplitude = 0.0;   // Used for the motion of the torso in the vertical direction during gait
  // Etc..
  walking_param_.arm_swing_gain = 1.0;                 // Arm swing during gait
  walking_param_.pelvis_offset = 0.0 * DEGREE2RADIAN;  // For gravitational compensation. Applied to the both legs..

  x_swap_phase_shift_ = M_PI;              // Are those really used??? What reference used here...?
  x_swap_amplitude_shift_ = 0;
  x_move_phase_shift_ = M_PI / 2;
  x_move_amplitude_shift_ = 0;
  y_swap_phase_shift_ = 0;
  y_swap_amplitude_shift_ = 0;
  y_move_phase_shift_ = M_PI / 2;
  z_swap_phase_shift_ = M_PI * 3 / 2;
  z_move_phase_shift_ = M_PI / 2;
  a_move_phase_shift_ = M_PI / 2;

  ctrl_running_ = false;     // for what? 
  real_running_ = false;     // for what? 
  time_ = 0;

  //              R_ARM_PITCH,  R_ARM_ROLL,   L_ARM_PITCH,    L_ARM_ROLL, 
  //              R_HIP_ROLL,   R_HIP_PITCH,  R_ANKLE_PITCH,  R_ANKLE_ROLL,
  //              L_HIP_ROLL,   L_HIP_PITCH,  L_ANKLE_PITCH,  L_ANKLE_ROLL;
  init_position_ <<      5.0,           0.0,           -5.0,           0.0,
                         0.0,           0.0,            0.0,           0.0,       
                         0.0,           0.0,            0.0,           0.0;
                                
  init_position_ *= DEGREE2RADIAN;

  ros::NodeHandle ros_node;

  std::string default_param_path = ros::package::getPath("robotis_engineer_walking_module") + "/config/param.yaml";
  ros_node.param<std::string>("walking_param_path", param_path_, default_param_path);

  loadWalkingParam(param_path_);

  updateTimeParam();
  updateMovementParam();
}

void WalkingModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("robotis/status", 1);

  /* ROS Service Callback Functions */    //communicate with what?
  ros::ServiceServer get_walking_param_server = ros_node.advertiseService("/robotis/walking/get_params",
                                                                          &WalkingModule::getWalkigParameterCallback,
                                                                          this);

  /* sensor topic subscribe */
  ros::Subscriber walking_command_sub = ros_node.subscribe("/robotis/walking/command", 0,
                                                           &WalkingModule::walkingCommandCallback, this);
  ros::Subscriber walking_param_sub = ros_node.subscribe("/robotis/walking/set_params", 0,
                                                         &WalkingModule::walkingParameterCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);   // for what?
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void WalkingModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Walking";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}

void WalkingModule::walkingCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  if(enable_ == false)
  {
    ROS_WARN("Walking module is not ready.");
    return;
  }

  if (msg->data == "start")
    startWalking();
  else if (msg->data == "stop")
    stop();
  else if (msg->data == "balance on")
    walking_param_.balance_enable = true;
  else if (msg->data == "balance off")
    walking_param_.balance_enable = false;
  else if (msg->data == "save")
    saveWalkingParam(param_path_);
}

void WalkingModule::walkingParameterCallback(const robotis_engineer_walking_module_msgs::WalkingParam::ConstPtr &msg)
{
  walking_param_ = *msg;
}

bool WalkingModule::getWalkigParameterCallback(robotis_engineer_walking_module_msgs::GetWalkingParam::Request &req,
                                               robotis_engineer_walking_module_msgs::GetWalkingParam::Response &res)
{
  res.parameters = walking_param_;

  return true;
}
// wSin stands for what???
double WalkingModule::wSin(double time, double period, double period_shift, double mag, double mag_shift)
{
  return mag * sin(2 * M_PI / period * time - period_shift) + mag_shift;
}

void WalkingModule::updateTimeParam()
{
  period_time_ = walking_param_.period_time;  // * 1000;   // s -> ms
  dsp_ratio_ = walking_param_.dsp_ratio;                   // double stance phase
  ssp_ratio_ = 1 - dsp_ratio_;                             // single stance phase

  x_swap_period_time_ = period_time_ / 2;                  // why are the below different from each other?
  x_move_period_time_ = period_time_ * ssp_ratio_;
  y_swap_period_time_ = period_time_;
  y_move_period_time_ = period_time_ * ssp_ratio_;
  z_swap_period_time_ = period_time_ / 2;
  z_move_period_time_ = period_time_ * ssp_ratio_ / 2;
  a_move_period_time_ = period_time_ * ssp_ratio_;

  ssp_time_ = period_time_ * ssp_ratio_;
  l_ssp_start_time_ = (1 - ssp_ratio_) * period_time_ / 4;
  l_ssp_end_time_ = (1 + ssp_ratio_) * period_time_ / 4;
  r_ssp_start_time_ = (3 - ssp_ratio_) * period_time_ / 4;
  r_ssp_end_time_ = (3 + ssp_ratio_) * period_time_ / 4;

  phase1_time_ = (l_ssp_start_time_ + l_ssp_end_time_) / 2;
  phase2_time_ = (l_ssp_end_time_ + r_ssp_start_time_) / 2; // swapping legs
  phase3_time_ = (r_ssp_start_time_ + r_ssp_end_time_) / 2;

  pelvis_offset_ = walking_param_.pelvis_offset;
  pelvis_swing_ = pelvis_offset_ * 0.35;                    // for what??.. why multiply 0.35
  arm_swing_gain_ = walking_param_.arm_swing_gain;
}
// should refer to what to understand the below??
void WalkingModule::updateMovementParam()
{
  // Forward/Back
  x_move_amplitude_ = walking_param_.x_move_amplitude;
  x_swap_amplitude_ = walking_param_.x_move_amplitude * walking_param_.step_fb_ratio; // for what?

  if (previous_x_move_amplitude_ == 0)                      // for the first step...
  {
    x_move_amplitude_ *= 0.5;
    x_swap_amplitude_ *= 0.5;
  }

  // Right/Left
  y_move_amplitude_ = walking_param_.y_move_amplitude / 2;    // for what?
  if (y_move_amplitude_ > 0)
    y_move_amplitude_shift_ = y_move_amplitude_;
  else
    y_move_amplitude_shift_ = -y_move_amplitude_;
  y_swap_amplitude_ = walking_param_.y_swap_amplitude + y_move_amplitude_shift_ * 0.04;  // for what... 0.04..

  z_move_amplitude_ = walking_param_.z_move_amplitude / 2;
  z_move_amplitude_shift_ = z_move_amplitude_ / 2;
  z_swap_amplitude_ = walking_param_.z_swap_amplitude;
  z_swap_amplitude_shift_ = z_swap_amplitude_;

  // Direction
  if (walking_param_.move_aim_on == false)         // aim on means what??? is it for rotation?
  {
    a_move_amplitude_ = walking_param_.angle_move_amplitude / 2;
    if (a_move_amplitude_ > 0)
      a_move_amplitude_shift_ = a_move_amplitude_;
    else
      a_move_amplitude_shift_ = -a_move_amplitude_;
  }
  else
  {
    a_move_amplitude_ = -walking_param_.angle_move_amplitude / 2;
    if (a_move_amplitude_ > 0)
      a_move_amplitude_shift_ = -a_move_amplitude_;
    else
      a_move_amplitude_shift_ = a_move_amplitude_;
  }
}
// not intuitive parameter names...
void WalkingModule::updatePoseParam()
{
  x_offset_ = walking_param_.init_x_offset;
  y_offset_ = walking_param_.init_y_offset;
  z_offset_ = walking_param_.init_z_offset;
  r_offset_ = walking_param_.init_roll_offset;
  p_offset_ = walking_param_.init_pitch_offset;
  a_offset_ = walking_param_.init_yaw_offset;
  hip_pitch_offset_ = walking_param_.hip_pitch_offset;
}

void WalkingModule::startWalking()
{
  ctrl_running_ = true;
  real_running_ = true;

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start walking");
}

void WalkingModule::stop()
{
  ctrl_running_ = false;
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Stop walking");
}
// below for waht??
bool WalkingModule::isRunning()
{
  return real_running_ || (walking_state_ == WalkingInitPose);
}

// default [angle : radian, length : m]    //used where...?
void WalkingModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                            std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  const double time_unit = control_cycle_msec_ * 0.001;  // ms -> s
  int joint_size = result_.size();
  double angle[joint_size];
  double balance_angle[joint_size];

  if (walking_state_ == WalkingInitPose)
  {
    int total_count = calc_joint_tra_.rows();
    for (int id = 1; id <= result_.size(); id++)
      target_position_.coeffRef(0, id) = calc_joint_tra_(init_pose_count_, id);

    init_pose_count_ += 1;
    if (init_pose_count_ >= total_count)
    {
      walking_state_ = WalkingReady;
      if (DEBUG)
        std::cout << "End moving to Init : " << init_pose_count_ << std::endl;
    }

  }
  else if (walking_state_ == WalkingReady || walking_state_ == WalkingEnable)
  {
    // present angle
    for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
        state_iter != result_.end(); state_iter++)
    {
      std::string _joint_name = state_iter->first;
      int joint_index = joint_table_[_joint_name];

      robotis_framework::Dynamixel *dxl = NULL;
      std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(_joint_name);
      if (dxl_it != dxls.end())
        dxl = dxl_it->second;
      else
        continue;

      goal_position_.coeffRef(0, joint_index) = dxl->dxl_state_->goal_position_;
    }

    processPhase(time_unit);

    // Compute Leg and Arm Angles
    bool get_angle = false;
    get_angle = computeLegAngle(&angle[4]);
    computeArmAngle(&angle[0]);



    // double rl_gyro_err = 0.0 - sensors["gyro_x"];
    // double fb_gyro_err = 0.0 - sensors["gyro_y"];

    // sensoryFeedback(rl_gyro_err, fb_gyro_err, balance_angle);

    double err_total = 0.0, err_max = 0.0;
    // set goal position
    for (int idx = 0; idx < result_.size(); idx++)
    {
      double goal_position = 0.0;
      if (get_angle == false && idx < result_.size())
        goal_position = goal_position_.coeff(0, idx);
      else
        // goal_position = init_position_.coeff(0, idx) + angle[idx] + balance_angle[idx];
        goal_position = init_position_.coeff(0, idx) + angle[idx];

      target_position_.coeffRef(0, idx) = goal_position;

      double err = fabs(target_position_.coeff(0, idx) - goal_position_.coeff(0, idx)) * RADIAN2DEGREE;
      if (err > err_max)
        err_max = err;
      err_total += err;
    }

    // ROS_INFO("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", 
    //       angle[0], angle[1], angle[2], angle[3], 
    //       angle[4], angle[5], angle[6], angle[7], 
    //       angle[8], angle[9], angle[10],angle[11]);


    // Check Enable
    if (walking_state_ == WalkingEnable && err_total > 5.0)
    {
      if (DEBUG)
        std::cout << "Check Err : " << err_max << std::endl;

      // make trajecotry for init pose
      int mov_time = err_max / 30;
      iniPoseTraGene(mov_time < 1 ? 1 : mov_time);

      // set target to goal
      target_position_ = goal_position_;

      walking_state_ = WalkingInitPose;

      ROS_WARN_STREAM_COND(DEBUG, "x_offset: " << walking_param_.init_x_offset);
      ROS_WARN_STREAM_COND(DEBUG, "y_offset: " << walking_param_.init_y_offset);
      ROS_WARN_STREAM_COND(DEBUG, "z_offset: " << walking_param_.init_z_offset);
      ROS_WARN_STREAM_COND(DEBUG, "roll_offset: " << walking_param_.init_roll_offset * RADIAN2DEGREE);
      ROS_WARN_STREAM_COND(DEBUG, "pitch_offset: " << walking_param_.init_pitch_offset * RADIAN2DEGREE);
      ROS_WARN_STREAM_COND(DEBUG, "yaw_offset: " << walking_param_.init_yaw_offset * RADIAN2DEGREE);
      ROS_WARN_STREAM_COND(DEBUG, "hip_pitch_offset: " << walking_param_.hip_pitch_offset * RADIAN2DEGREE);
      ROS_WARN_STREAM_COND(DEBUG, "period_time: " << walking_param_.period_time * 1000);
      ROS_WARN_STREAM_COND(DEBUG, "dsp_ratio: " << walking_param_.dsp_ratio);
      ROS_WARN_STREAM_COND(DEBUG, "step_forward_back_ratio: " << walking_param_.step_fb_ratio);
      ROS_WARN_STREAM_COND(DEBUG, "foot_height: " << walking_param_.z_move_amplitude);
      ROS_WARN_STREAM_COND(DEBUG, "swing_right_left: " << walking_param_.y_swap_amplitude);
      ROS_WARN_STREAM_COND(DEBUG, "swing_top_down: " << walking_param_.z_swap_amplitude);
      ROS_WARN_STREAM_COND(DEBUG, "pelvis_offset: " << walking_param_.pelvis_offset * RADIAN2DEGREE);
      ROS_WARN_STREAM_COND(DEBUG, "arm_swing_gain: " << walking_param_.arm_swing_gain);
      ROS_WARN_STREAM_COND(DEBUG, "balance_hip_roll_gain: " << walking_param_.balance_hip_roll_gain);
      ROS_WARN_STREAM_COND(DEBUG, "balance_knee_gain: " << walking_param_.balance_knee_gain);
      ROS_WARN_STREAM_COND(DEBUG, "balance_ankle_roll_gain: " << walking_param_.balance_ankle_roll_gain);
      ROS_WARN_STREAM_COND(DEBUG, "balance_ankle_pitch_gain: " << walking_param_.balance_ankle_pitch_gain);
      ROS_WARN_STREAM_COND(DEBUG, "balance : " << (walking_param_.balance_enable ? "TRUE" : "FALSE"));
    }
    else
    {
      walking_state_ = WalkingReady;
    }
  }

  // set result
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
      state_it != result_.end(); state_it++)
  {
    std::string joint_name = state_it->first;
    int joint_index = joint_table_[joint_name];

    result_[joint_name]->goal_position_ = target_position_.coeff(0, joint_index);

    // Todo : setting PID gain to the leg joints
    // result_[joint_name]->position_p_gain_ = walking_param_.p_gain;
    // result_[joint_name]->position_i_gain_ = walking_param_.i_gain;
    // result_[joint_name]->position_d_gain_ = walking_param_.d_gain;
  }

  if (real_running_ == true)
  {
    time_ += time_unit;
    if (time_ >= period_time_)
    {
      time_ = 0;
      previous_x_move_amplitude_ = walking_param_.x_move_amplitude * 0.5;
    }
  }
}

void WalkingModule::processPhase(const double &time_unit)
{
  // Update walk parameters
  if (time_ == 0)
  {
    updateTimeParam();
    phase_ = PHASE0;
    if (ctrl_running_ == false)
    {
      if (x_move_amplitude_ == 0 && y_move_amplitude_ == 0 && a_move_amplitude_ == 0)
      {
        real_running_ = false;
      }
      else
      {
        // set walking param to init
        walking_param_.x_move_amplitude = 0;
        walking_param_.y_move_amplitude = 0;
        walking_param_.angle_move_amplitude = 0;

        previous_x_move_amplitude_ = 0;
      }
    }
  }
  // the position of left foot is the highest.
  else if (time_ >= (phase1_time_ - time_unit / 2) && time_ < (phase1_time_ + time_unit / 2))  
  {
    updateMovementParam();
    phase_ = PHASE1;
  }
  // middle of double support state
  else if (time_ >= (phase2_time_ - time_unit / 2) && time_ < (phase2_time_ + time_unit / 2))  
  {
    updateTimeParam();

    time_ = phase2_time_;
    phase_ = PHASE2;
    if (ctrl_running_ == false)
    {
      if (x_move_amplitude_ == 0 && y_move_amplitude_ == 0 && a_move_amplitude_ == 0)
      {
        real_running_ = false;
      }
      else
      {
        // set walking param to init
        walking_param_.x_move_amplitude = previous_x_move_amplitude_;
        walking_param_.y_move_amplitude = 0;
        walking_param_.angle_move_amplitude = 0;
      }
    }
  }
  else if (time_ >= (phase3_time_ - time_unit / 2) && time_ < (phase3_time_ + time_unit / 2))  // the position of right foot is the highest.
  {
    updateMovementParam();
    phase_ = PHASE3;
  }
}

bool WalkingModule::computeLegAngle(double *leg_angle)
{
  Pose3D swap, right_leg_move, left_leg_move;
  double pelvis_offset_r, pelvis_offset_l;
  double ep[12]; // endpoints

  updatePoseParam();

  // Compute endpoints     // swap stands for what?  
  // why the torso behave like the below? ??>...<??
  // = x_move_amplitude * fb * sin(2*PI/(period/2)*time - PI)                     ??   0 at period/4, period*2/4
  swap.x = wSin(time_, x_swap_period_time_, x_swap_phase_shift_, x_swap_amplitude_, x_swap_amplitude_shift_);
  // = y_swap_amplitude_ * sin(2*PI/(period)*time)                                 ??  0 at 0, period*2/4
  swap.y = wSin(time_, y_swap_period_time_, y_swap_phase_shift_, y_swap_amplitude_, y_swap_amplitude_shift_);
  // = z_swap_amplitude_ * sin(2*PI/(period/2)*time  - PI*3/2) + z_swap_amplitude_ ??  0 at period/8, period*3/8
  swap.z = wSin(time_, z_swap_period_time_, z_swap_phase_shift_, z_swap_amplitude_, z_swap_amplitude_shift_);
  swap.roll = 0.0;
  swap.pitch = 0.0;
  swap.yaw = 0.0;

  // no change during this time...
  if (time_ <= l_ssp_start_time_)     
  {
    // = x_move_amplitude * sin(-PI/2)
    left_leg_move.x = wSin(l_ssp_start_time_, x_move_period_time_,
                           x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_, 
                           x_move_amplitude_, x_move_amplitude_shift_); 
    // = y_move_amplitude * sin(-PI/2) + y_move_amplitude/2
    left_leg_move.y = wSin(l_ssp_start_time_, y_move_period_time_,
                           y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_, 
                           y_move_amplitude_, y_move_amplitude_shift_); 
    // = z_move_amplitude * sin(-PI/2) + z_move_amplitude_
    left_leg_move.z = wSin(l_ssp_start_time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, 
                           z_move_amplitude_, z_move_amplitude_shift_);
    // = a_move_amplitude * sin(-PI/2) + a_move_amplitude/2
    left_leg_move.yaw = wSin(l_ssp_start_time_, a_move_period_time_,
                             a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                             a_move_amplitude_, a_move_amplitude_shift_);
    // = -x_move_amplitude * sin(-PI/2)
    right_leg_move.x = wSin(l_ssp_start_time_, x_move_period_time_,
                            x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_,
                            -x_move_amplitude_, -x_move_amplitude_shift_);
    // = -y_move_amplitude * sin(-PI/2) - y_move_amplitude/2
    right_leg_move.y = wSin(l_ssp_start_time_, y_move_period_time_,
                            y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_,
                            -y_move_amplitude_, -y_move_amplitude_shift_);
    // = z_move_amplitude * sin(-PI/2) + z_move_amplitude_
    right_leg_move.z = wSin(r_ssp_start_time_, z_move_period_time_,
                            z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, 
                            z_move_amplitude_, z_move_amplitude_shift_);
    // = -a_move_amplitude * sin(-PI/2) - a_move_amplitude/2
    right_leg_move.yaw = wSin(l_ssp_start_time_, a_move_period_time_,
                              a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                              -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = 0;
    pelvis_offset_r = 0;
  }
  else if (time_ <= l_ssp_end_time_)    
  {
    left_leg_move.x = wSin(time_, x_move_period_time_,
                           x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_, 
                           x_move_amplitude_, x_move_amplitude_shift_);
    left_leg_move.y = wSin(time_, y_move_period_time_,
                           y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_, 
                           y_move_amplitude_, y_move_amplitude_shift_);
    left_leg_move.z = wSin(time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, 
                           z_move_amplitude_, z_move_amplitude_shift_);
    left_leg_move.yaw = wSin(time_, a_move_period_time_,
                             a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                             a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x = wSin(time_, x_move_period_time_,
                            x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_,
                            -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y = wSin(time_, y_move_period_time_,
                            y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_,
                            -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z = wSin(r_ssp_start_time_, z_move_period_time_,
                            z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, 
                            z_move_amplitude_, z_move_amplitude_shift_);
    right_leg_move.yaw = wSin(time_, a_move_period_time_,
                              a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                              -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = wSin(time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, 
                           pelvis_swing_ / 2, pelvis_swing_ / 2);
    pelvis_offset_r = wSin(time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_,
                           -pelvis_offset_ / 2, -pelvis_offset_ / 2);
  }
  else if (time_ <= r_ssp_start_time_)
  {
    left_leg_move.x = wSin(l_ssp_end_time_, x_move_period_time_,
                           x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_, 
                           x_move_amplitude_, x_move_amplitude_shift_);   
    left_leg_move.y = wSin(l_ssp_end_time_, y_move_period_time_,
                           y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_, 
                           y_move_amplitude_, y_move_amplitude_shift_);
    left_leg_move.z = wSin(l_ssp_end_time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, 
                           z_move_amplitude_, z_move_amplitude_shift_);
    left_leg_move.yaw = wSin(l_ssp_end_time_, a_move_period_time_,
                             a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                             a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x = wSin(l_ssp_end_time_, x_move_period_time_,
                            x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_,
                            -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y = wSin(l_ssp_end_time_, y_move_period_time_,
                            y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_,
                            -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z = wSin(r_ssp_start_time_, z_move_period_time_,
                            z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, 
                            z_move_amplitude_, z_move_amplitude_shift_);
    right_leg_move.yaw = wSin(l_ssp_end_time_, a_move_period_time_,
                              a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                              -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = 0;
    pelvis_offset_r = 0;
  }
  else if (time_ <= r_ssp_end_time_)
  {
    left_leg_move.x = wSin(time_, x_move_period_time_,
                           x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                           x_move_amplitude_, x_move_amplitude_shift_);
    left_leg_move.y = wSin(time_, y_move_period_time_,
                           y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                           y_move_amplitude_, y_move_amplitude_shift_);
    left_leg_move.z = wSin(l_ssp_end_time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, 
                           z_move_amplitude_, z_move_amplitude_shift_);
    left_leg_move.yaw = wSin(time_, a_move_period_time_,
                             a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                             a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x = wSin(time_, x_move_period_time_,
                            x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                            -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y = wSin(time_, y_move_period_time_,
                            y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                            -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z = wSin(time_, z_move_period_time_,
                            z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, 
                            z_move_amplitude_, z_move_amplitude_shift_);
    right_leg_move.yaw = wSin(time_, a_move_period_time_,
                              a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                              -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = wSin(time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, 
                           pelvis_offset_ / 2, pelvis_offset_ / 2);
    pelvis_offset_r = wSin(time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, 
                           -pelvis_swing_ / 2, -pelvis_swing_ / 2);
  }
  else
  {
    left_leg_move.x = wSin(r_ssp_end_time_, x_move_period_time_,
                           x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                           x_move_amplitude_, x_move_amplitude_shift_);
    left_leg_move.y = wSin(r_ssp_end_time_, y_move_period_time_,
                           y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                           y_move_amplitude_, y_move_amplitude_shift_);
    left_leg_move.z = wSin(l_ssp_end_time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, 
                           z_move_amplitude_, z_move_amplitude_shift_);
    left_leg_move.yaw = wSin(r_ssp_end_time_, a_move_period_time_,
                             a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                             a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x = wSin(r_ssp_end_time_, x_move_period_time_,
                            x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                            -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y = wSin(r_ssp_end_time_, y_move_period_time_,
                            y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                            -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z = wSin(r_ssp_end_time_, z_move_period_time_,
                            z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, 
                            z_move_amplitude_, z_move_amplitude_shift_);
    right_leg_move.yaw = wSin(r_ssp_end_time_, a_move_period_time_,
                              a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                              -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = 0;
    pelvis_offset_r = 0;
  }

  left_leg_move.roll = 0;
  left_leg_move.pitch = 0;
  right_leg_move.roll = 0;
  right_leg_move.pitch = 0;

  // double leg_height = robotis_engineer_kd_->thigh_length_m_ + robotis_engineer_kd_->calf_length_m_ + robotis_engineer_kd_->ankle_length_m_;
  double leg_height = 200.0*0.001;     // Height from the sole to the hip Roll
  // mm, rad
  // For Right Leg
  ep[0] = swap.x + right_leg_move.x + x_offset_;
  // ep[1] = swap.y + right_leg_move.y - y_offset_ / 2;
  ep[1] = swap.y + right_leg_move.y;
  // ep[2] = swap.z + right_leg_move.z + z_offset_ - leg_height;
  ep[2] = swap.z + right_leg_move.z - leg_height;
  ep[3] = swap.roll + right_leg_move.roll - r_offset_ / 2;
  ep[4] = swap.pitch + right_leg_move.pitch + p_offset_;
  ep[5] = swap.yaw + right_leg_move.yaw - a_offset_ / 2;

  // For Left Leg
  ep[6] = swap.x + left_leg_move.x + x_offset_;
  // ep[7] = swap.y + left_leg_move.y + y_offset_ / 2;
  ep[7] = swap.y + left_leg_move.y;
  // ep[8] = swap.z + left_leg_move.z + z_offset_ - leg_height;
  ep[8] = swap.z + left_leg_move.z - leg_height;
  ep[9] = swap.roll + left_leg_move.roll + r_offset_ / 2;
  ep[10] = swap.pitch + left_leg_move.pitch + p_offset_;
  ep[11] = swap.yaw + left_leg_move.yaw + a_offset_ / 2;

  // ROS_INFO("%f, %f", ep[1], ep[7]); 

  // Compute DXL Angles 
  // Right Leg       
  // if (robotis_engineer_kd_->computeIK(&leg_angle[0], ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]) == false)
  if (robotis_engineer_kd_->calcInverseKinematicsForRightLeg(&leg_angle[0], ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]) == false)
  {
    printf("IK not Solved EPR : %f %f %f %f %f %f\n", ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]);
    return false;
  }
                       
  // Left Leg       
  // if (robotis_engineer_kd_->computeIK(&leg_angle[4], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]) == false)
  if (robotis_engineer_kd_->calcInverseKinematicsForLeftLeg(&leg_angle[4], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]) == false)
  {
    printf("IK not Solved EPL : %f %f %f %f %f %f\n", ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]);
    return false;
  }

  ROS_INFO("%f, %f, %f, %f, %f, %f ,%f, %f", leg_angle[0], leg_angle[1], leg_angle[2], leg_angle[3], leg_angle[4], leg_angle[5], leg_angle[6], leg_angle[7]);

  // Add offset values to the DXL Angles 
  for (int i = 0; i < 12; i++)
  {
    // Offset : rad
    double offset = 0;

    if (i == joint_table_["r_hip_roll"])  // R_HIP_ROLL
      offset += robotis_engineer_kd_->getJointDirection("r_hip_roll") * pelvis_offset_r;
    else if (i == joint_table_["l_hip_roll"])  // L_HIP_ROLL
      offset += robotis_engineer_kd_->getJointDirection("l_hip_roll") * pelvis_offset_l;
    else if (i == joint_table_["r_hip_pitch"])
      offset -= robotis_engineer_kd_->getJointDirection("r_hip_pitch") * hip_pitch_offset_;
    else if (i == joint_table_["l_hip_pitch"])  // R_HIP_PITCH or L_HIP_PITCH
      offset -= robotis_engineer_kd_->getJointDirection("l_hip_pitch") * hip_pitch_offset_;

    leg_angle[i-4] += offset; // leg_angle[i-4] considering the first 4 joints for arms
  }

  return true;
}

void WalkingModule::computeArmAngle(double *arm_angle)
{
  // Compute arm swing
  if (x_move_amplitude_ == 0)
  {
    arm_angle[0] = 0;  // Right Arm Pitch
    arm_angle[1] = 0;  // Right Arm Roll
    arm_angle[2] = 0;  // Left Arm Pitch
    arm_angle[3] = 0;  // Left Arm Roll
  }
  else
  {     
    arm_angle[0] = wSin(time_, period_time_, M_PI * 1.5, -x_move_amplitude_ * arm_swing_gain_ * 1000, 0) 
                        * robotis_engineer_kd_->getJointDirection("r_shoulder_pitch") * DEGREE2RADIAN;
    arm_angle[1] = 0;
    arm_angle[2] = wSin(time_, period_time_, M_PI * 1.5, x_move_amplitude_ * arm_swing_gain_ * 1000, 0) 
                        * robotis_engineer_kd_->getJointDirection("l_shoulder_pitch") * DEGREE2RADIAN;
    // arm_angle[2] = 0;
    arm_angle[3] = 0;
  }

  // ROS_INFO("x_move_amplitude_ %f", x_move_amplitude_);
  // ROS_INFO("RIGHT %f", arm_angle[0]);
  // ROS_INFO("LEFT  %f", arm_angle[2]);
}

void WalkingModule::sensoryFeedback(const double &rlGyroErr, const double &fbGyroErr, double *balance_angle)
{
  // adjust balance offset
  if (walking_param_.balance_enable == false)
    return;

  double internal_gain = 0.05;

  balance_angle[joint_table_["r_hip_roll"]] =  robotis_engineer_kd_->getJointDirection("r_hip_roll") * internal_gain
      * rlGyroErr * walking_param_.balance_hip_roll_gain;  // R_HIP_ROLL
  balance_angle[joint_table_["l_hip_roll"]] =  robotis_engineer_kd_->getJointDirection("l_hip_roll") * internal_gain
      * rlGyroErr * walking_param_.balance_hip_roll_gain;  // L_HIP_ROLL

  balance_angle[joint_table_["r_knee"]] = - robotis_engineer_kd_->getJointDirection("r_knee") * internal_gain
      * fbGyroErr * walking_param_.balance_knee_gain;  // R_KNEE
  balance_angle[joint_table_["l_knee"]] = - robotis_engineer_kd_->getJointDirection("l_knee") * internal_gain
      * fbGyroErr * walking_param_.balance_knee_gain;  // L_KNEE

  balance_angle[joint_table_["r_ankle_pitch"]] = - robotis_engineer_kd_->getJointDirection("r_ankle_pitch")
      * internal_gain * fbGyroErr * walking_param_.balance_ankle_pitch_gain;  // R_ANKLE_PITCH
  balance_angle[joint_table_["l_ankle_pitch"]] = - robotis_engineer_kd_->getJointDirection("l_ankle_pitch")
      * internal_gain * fbGyroErr * walking_param_.balance_ankle_pitch_gain;  // L_ANKLE_PITCH

  balance_angle[joint_table_["r_ankle_roll"]] = - robotis_engineer_kd_->getJointDirection("r_ankle_roll") * internal_gain
      * rlGyroErr * walking_param_.balance_ankle_roll_gain;  // R_ANKLE_ROLL
  balance_angle[joint_table_["l_ankle_roll"]] = - robotis_engineer_kd_->getJointDirection("l_ankle_roll") * internal_gain
      * rlGyroErr * walking_param_.balance_ankle_roll_gain;  // L_ANKLE_ROLL
}

void WalkingModule::loadWalkingParam(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // Load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  // Parse movement time
  walking_param_.init_x_offset = doc["x_offset"].as<double>();
  walking_param_.init_y_offset = doc["y_offset"].as<double>();
  walking_param_.init_z_offset = doc["z_offset"].as<double>();
  walking_param_.init_roll_offset = doc["roll_offset"].as<double>() * DEGREE2RADIAN;
  walking_param_.init_pitch_offset = doc["pitch_offset"].as<double>() * DEGREE2RADIAN;
  walking_param_.init_yaw_offset = doc["yaw_offset"].as<double>() * DEGREE2RADIAN;
  walking_param_.hip_pitch_offset = doc["hip_pitch_offset"].as<double>() * DEGREE2RADIAN;
  // Time
  walking_param_.period_time = doc["period_time"].as<double>() * 0.001;    // ms -> s
  walking_param_.dsp_ratio = doc["dsp_ratio"].as<double>();
  walking_param_.step_fb_ratio = doc["step_forward_back_ratio"].as<double>();
  // Walking
  // walking_param_.x_move_amplitude
  // walking_param_.y_move_amplitude
  walking_param_.z_move_amplitude = doc["foot_height"].as<double>();
  // walking_param_.angle_move_amplitude
  // walking_param_.move_aim_on

  // Balance
  // walking_param_.balance_enable
  walking_param_.balance_hip_roll_gain = doc["balance_hip_roll_gain"].as<double>();
  walking_param_.balance_knee_gain = doc["balance_knee_gain"].as<double>();
  walking_param_.balance_ankle_roll_gain = doc["balance_ankle_roll_gain"].as<double>();
  walking_param_.balance_ankle_pitch_gain = doc["balance_ankle_pitch_gain"].as<double>();
  walking_param_.y_swap_amplitude = doc["swing_right_left"].as<double>();
  walking_param_.z_swap_amplitude = doc["swing_top_down"].as<double>();
  walking_param_.pelvis_offset = doc["pelvis_offset"].as<double>() * DEGREE2RADIAN;
  walking_param_.arm_swing_gain = doc["arm_swing_gain"].as<double>();

  // gain
  walking_param_.p_gain = doc["p_gain"].as<int>();   // why int??? not double..
  walking_param_.i_gain = doc["i_gain"].as<int>();
  walking_param_.d_gain = doc["d_gain"].as<int>();
}

void WalkingModule::saveWalkingParam(std::string &path)
{
  YAML::Emitter out_emitter;

  out_emitter << YAML::BeginMap;
  out_emitter << YAML::Key << "x_offset" << YAML::Value << walking_param_.init_x_offset;
  out_emitter << YAML::Key << "y_offset" << YAML::Value << walking_param_.init_y_offset;
  out_emitter << YAML::Key << "z_offset" << YAML::Value << walking_param_.init_z_offset;
  out_emitter << YAML::Key << "roll_offset" << YAML::Value << walking_param_.init_roll_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "pitch_offset" << YAML::Value << walking_param_.init_pitch_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "yaw_offset" << YAML::Value << walking_param_.init_yaw_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "hip_pitch_offset" << YAML::Value << walking_param_.hip_pitch_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "period_time" << YAML::Value << walking_param_.period_time * 1000;
  out_emitter << YAML::Key << "dsp_ratio" << YAML::Value << walking_param_.dsp_ratio;
  out_emitter << YAML::Key << "step_forward_back_ratio" << YAML::Value << walking_param_.step_fb_ratio;
  out_emitter << YAML::Key << "foot_height" << YAML::Value << walking_param_.z_move_amplitude;
  out_emitter << YAML::Key << "swing_right_left" << YAML::Value << walking_param_.y_swap_amplitude;
  out_emitter << YAML::Key << "swing_top_down" << YAML::Value << walking_param_.z_swap_amplitude;
  out_emitter << YAML::Key << "pelvis_offset" << YAML::Value << walking_param_.pelvis_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "arm_swing_gain" << YAML::Value << walking_param_.arm_swing_gain;
  out_emitter << YAML::Key << "balance_hip_roll_gain" << YAML::Value << walking_param_.balance_hip_roll_gain;
  out_emitter << YAML::Key << "balance_knee_gain" << YAML::Value << walking_param_.balance_knee_gain;
  out_emitter << YAML::Key << "balance_ankle_roll_gain" << YAML::Value << walking_param_.balance_ankle_roll_gain;
  out_emitter << YAML::Key << "balance_ankle_pitch_gain" << YAML::Value << walking_param_.balance_ankle_pitch_gain;

  out_emitter << YAML::Key << "p_gain" << YAML::Value << walking_param_.p_gain;
  out_emitter << YAML::Key << "i_gain" << YAML::Value << walking_param_.i_gain;
  out_emitter << YAML::Key << "d_gain" << YAML::Value << walking_param_.d_gain;
  out_emitter << YAML::EndMap;

  // output to file
  std::ofstream fout(path.c_str());
  fout << out_emitter.c_str();
}

void WalkingModule::onModuleEnable()
{
  walking_state_ = WalkingEnable;
  ROS_INFO("Walking Enable");
}

void WalkingModule::onModuleDisable()
{
  ROS_INFO("Walking Disable");
  walking_state_ = WalkingDisable;
}

void WalkingModule::iniPoseTraGene(double mov_time)
{
  double smp_time = control_cycle_msec_ * 0.001;
  int all_time_steps = int(mov_time / smp_time) + 1;
  calc_joint_tra_.resize(all_time_steps, result_.size() + 1);

  for (int id = 0; id <= result_.size(); id++)
  {
    double ini_value = goal_position_.coeff(0, id);
    double tar_value = target_position_.coeff(0, id);

    Eigen::MatrixXd tra;

    tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0, smp_time, mov_time);

    calc_joint_tra_.block(0, id, all_time_steps, 1) = tra;
  }

  if(DEBUG)
    std::cout << "Generate Trajecotry : " << mov_time << "s [" << all_time_steps << "]" << std::endl;

  init_pose_count_ = 0;
}
}
