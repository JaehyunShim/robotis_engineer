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

/* Author: Kayman Jung */

#ifndef ROBOTIS_ENGINEER_CONTROL_GUI_MAIN_WINDOW_H
#define ROBOTIS_ENGINEER_CONTROL_GUI_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#endif
/*****************************************************************************
** Namespace
*****************************************************************************/
namespace robotis_engineer
{

#define DEGREE2RADIAN     (M_PI / 180.0)
#define RADIAN2DEGREE     (180.0 / M_PI)

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
Q_OBJECT

 public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();

  void readSettings();  // Load up qt program settings at startup
  void writeSettings();  // Save qt program settings when closing

  void closeEvent(QCloseEvent *event);  // Overloaded function
  void showNoMasterMessage();

 public Q_SLOTS:

  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
  void on_actionAbout_triggered();
  void on_button_clear_log_clicked(bool check);
  void on_button_init_pose_clicked(bool check);

  // Arm Control
  void on_button_arm_center_clicked(bool check);

  // // Head Control
  // void on_button_head_center_clicked(bool check);

  // // Waist Control
  // void on_button_waist_center_clicked(bool check);

  // Walking Control
  void on_button_init_gyro_clicked(bool check);
  void on_button_walking_start_clicked(bool check);
  void on_button_walking_stop_clicked(bool check);

  void on_button_param_refresh_clicked(bool check);
  void on_button_param_apply_clicked(bool check);
  void on_button_param_save_clicked(bool check);

  /******************************************
  ** Manual connections
  *******************************************/
  void updateLoggingView();  // no idea why this can't connect automatically
  void setMode(bool check);
  void updateCurrentJointMode(std::vector<int> mode);
  void setMode(QString mode_name);

  // Arm Control
  void updateArmAngles(double r_shoulder_pitch, double r_shoulder_roll, double l_shoulder_pitch, double l_shoulder_roll);

  // Walking
  void updateWalkingParams(robotis_engineer_walking_module_msgs::WalkingParam params);
  void walkingCommandShortcut();

 protected Q_SLOTS:     //for what??
  void setArmAngle();  

 private:
  enum Motion_Index
  {
    InitPose = 1,
    WalkingReady = 9,
    GetUpFront = 122,
    GetUpBack = 123,
    RightKick = 121,
    LeftKick = 120,
    Ceremony = 85,
    InitposeJH = 210,
  };

  void setUserShortcut();
  void initModeUnit();
  void initMotionUnit();

  void updateModuleUI();
  void setArmAngle(double r_shoulder_pitch, double r_shoulder_roll, double l_shoulder_pitch, double l_shoulder_roll);
  void applyWalkingParams();

  Ui::MainWindowDesign ui_;
  QNode qnode_;
  bool debug_;

  bool is_updating_;
  bool is_walking_;
  std::map<std::string, QList<QWidget *> > module_ui_table_;
};

}  // namespace robotis_engineer

#endif // ROBOTIS_ENGINEER_CONTROL_GUI_MAIN_WINDOW_H
