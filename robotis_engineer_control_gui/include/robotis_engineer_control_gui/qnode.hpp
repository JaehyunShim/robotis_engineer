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

/* Author: Kayman Jung, Ryan Shim */

/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef ROBOTIS_ENGINEER_CONTROL_GUI_QNODE_HPP_
#define ROBOTIS_ENGINEER_CONTROL_GUI_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN

#include <string>
#include <sstream>

#include <QThread>
#include <QStringListModel>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <interactive_markers/interactive_marker_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/GetJointModule.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

// walking demo
#include "robotis_engineer_walking_module_msgs/WalkingParam.h"
#include "robotis_engineer_walking_module_msgs/GetWalkingParam.h"
#include "robotis_engineer_walking_module_msgs/SetWalkingParam.h"

#endif

#define DEG2RAD   (M_PI / 180.0)
#define RAD2DEG   (180.0 / M_PI)
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robotis_engineer
{

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread
{
  Q_OBJECT
public:

  enum LogLevel
  {
    Debug = 0,
    Info = 1,
    Warn = 2,
    Error = 3,
    Fatal = 4
  };

  QNode(int argc, char** argv);
  virtual ~QNode();

  bool init();
  void run();

  QStringListModel* loggingModel()
  {
    return &logging_model_;
  }
  void log(const LogLevel &level, const std::string &msg, std::string sender = "Demo");
  void clearLog();
  void assemble_lidar();
  void setJointControlMode(const robotis_controller_msgs::JointCtrlModule &msg);
  void setControlMode(const std::string &mode);
  bool getJointNameFromID(const int &id, std::string &joint_name);
  bool getIDFromJointName(const std::string &joint_name, int &id);
  bool getIDJointNameFromIndex(const int &index, int &id, std::string &joint_name);
  std::string getModeName(const int &index);
  int getModeIndex(const std::string &mode_name);
  int getModeSize();
  int getJointSize();
  void clearUsingModule();
  bool isUsingModule(std::string module_name);
  void moveInitPose();
  void init_default_demo(ros::NodeHandle &ros_node);

  // Arm control
  void setArmJoint(double r_shoulder_pitch, double r_shoulder_roll, double l_shoulder_pitch, double l_shoulder_roll);

  // Walking
  void setWalkingCommand(const std::string &command);
  void refreshWalkingParam();
  void saveWalkingParam();
  void applyWalkingParam(const robotis_engineer_walking_module_msgs::WalkingParam &walking_param);
  void initGyro();

  // Demo
  void setDemoCommand(const std::string &command);
  void setActionModuleBody();
  void setModuleToDemo();

  // Interactive marker
  void makeInteractiveMarker(const geometry_msgs::Pose& marker_pose);
  bool updateInteractiveMarker(const geometry_msgs::Pose& pose);
  void getInteractiveMarkerPose();
  void clearInteractiveMarker();

  std::map<int, std::string> module_table_;
  std::map<int, std::string> motion_table_;
  std::map<int, int> motion_shortcut_table_;

public Q_SLOTS:
  void getJointControlMode();
  void playMotion(int motion_index);  // used to play motions written in the motion file (/action_module/data/motion.bin)

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();
  void updateCurrentJointControlMode(std::vector<int> mode);

  // Arm
  void updateArmAngles(double r_shoulder_pitch, double r_shoulder_roll, double l_shoulder_pitch, double l_shoulder_roll);

  // Walking
  void updateWalkingParameters(robotis_engineer_walking_module_msgs::WalkingParam params);

  // Interactive marker
  void updateDemoPoint(const geometry_msgs::Point point);
  void updateDemoPose(const geometry_msgs::Pose pose);

private:
  void parseJointNameFromYaml(const std::string &path);   // parse joints on the main tab
  void parseMotionMapFromYaml(const std::string &path);   // parse motions on the motion tab
  void refreshCurrentJointControlCallback(const robotis_controller_msgs::JointCtrlModule::ConstPtr &msg);
  void updateArmJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void updateHeadJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void updateWaistJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg);

  // interactive marker
  void pointStampedCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
  void interactiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  // localization
  bool transformPose(const std::string &from_id, const std::string &to_id, const geometry_msgs::Pose &from_pose,
                     geometry_msgs::Pose &to_pose, bool inverse = false);

  int init_argc_;
  char** init_argv_;
  bool debug_;
  double body_height_;

  // interactive marker
  ros::Subscriber rviz_clicked_point_sub_;
  std::string frame_id_;
  std::string marker_name_;
  geometry_msgs::Pose pose_from_ui_;
  geometry_msgs::Pose current_pose_;
  geometry_msgs::Pose curr_pose_msg_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
  boost::shared_ptr<tf::TransformListener> tf_listener_;

  robotis_engineer_walking_module_msgs::WalkingParam walking_param_;

  ros::Publisher init_pose_pub_;
  ros::Publisher module_control_pub_;
  ros::Publisher module_control_preset_pub_;
  ros::Publisher init_gyro_pub_;
  ros::Subscriber status_msg_sub_;
  ros::Subscriber init_ft_foot_sub_;
  ros::Subscriber both_ft_foot_sub_;
  ros::Subscriber current_module_control_sub_;
  ros::ServiceClient get_module_control_client_;

  // Arm
  ros::Publisher set_arm_joint_angle_pub_;
  ros::Subscriber current_arm_joint_states_sub_;

  // Walking
  ros::Publisher set_walking_command_pub;
  ros::Publisher set_walking_param_pub;
  ros::ServiceClient get_walking_param_client_;

  // Action
  ros::Publisher motion_index_pub_;

  // Demo
  ros::Publisher demo_command_pub_;

  ros::Time start_time_;

  QStringListModel logging_model_;
  std::map<int, std::string> id_joint_table_;
  std::map<std::string, int> joint_id_table_;

  std::map<int, std::string> index_mode_table_;
  std::map<std::string, int> mode_index_table_;
  std::map<std::string, bool> using_mode_table_;
};

}  // namespace robotis_engineer

template<typename T>
T deg2rad(T deg)
{
  return deg * M_PI / 180;
}

template<typename T>
T rad2deg(T rad)
{
  return rad * 180 / M_PI;
}
#endif /* ROBOTIS_ENGINEER_CONTROL_GUI_QNODE_HPP_ */
