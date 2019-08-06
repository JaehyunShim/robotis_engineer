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
** Includes
*****************************************************************************/
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "robotis_engineer_control_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace robotis_engineer
{

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/
MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent),
      qnode_(argc, argv),
      is_updating_(false),
      is_walking_(false)
{
  // code to DEBUG
  debug_ = false;

  if (argc >= 2)
  {
    std::string arg_code(argv[1]);
    if (arg_code == "debug")
      debug_ = true;
    else
      debug_ = false;
  }

  ui_.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui_.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));  // qApp is a global variable for the application

  readSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui_.tab_manager->setCurrentIndex(0);  // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode_, SIGNAL(rosShutdown()), this, SLOT(close()));

  qRegisterMetaType<std::vector<int> >("std::vector<int>");
  QObject::connect(&qnode_, SIGNAL(updateCurrentJointControlMode(std::vector<int>)), this,
                   SLOT(updateCurrentJointMode(std::vector<int>)));
  QObject::connect(&qnode_, SIGNAL(updateArmAngles(double,double,double,double)), this, SLOT(updateArmAngles(double,double,double,double)));

  QObject::connect(ui_.r_shoulder_roll_slider, SIGNAL(valueChanged(int)), this, SLOT(setArmAngle()));
  QObject::connect(ui_.r_shoulder_pitch_slider, SIGNAL(valueChanged(int)), this, SLOT(setArmAngle()));
  QObject::connect(ui_.l_shoulder_roll_slider, SIGNAL(valueChanged(int)), this, SLOT(setArmAngle()));
  QObject::connect(ui_.l_shoulder_pitch_slider, SIGNAL(valueChanged(int)), this, SLOT(setArmAngle()));


  qRegisterMetaType<robotis_engineer_walking_module_msgs::WalkingParam>("robotis_engineer_walking_params");
  QObject::connect(&qnode_, SIGNAL(updateWalkingParameters(robotis_engineer_walking_module_msgs::WalkingParam)), this,
                   SLOT(updateWalkingParams(robotis_engineer_walking_module_msgs::WalkingParam)));

  /*********************
  ** Logging
  **********************/
  ui_.view_logging->setModel(qnode_.loggingModel());
  QObject::connect(&qnode_, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  /*********************
  ** Auto Start
  **********************/
  qnode_.init();
  initModeUnit();
  setUserShortcut();
  updateModuleUI();
}

MainWindow::~MainWindow()
{}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/
void MainWindow::showNoMasterMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */
void MainWindow::on_button_clear_log_clicked(bool check)
{
  qnode_.clearLog();
}
void MainWindow::on_button_init_pose_clicked(bool check)
{
  qnode_.moveInitPose();
}

// Arm Control
void MainWindow::on_button_arm_center_clicked(bool check)
{
  qnode_.log(QNode::Info, "Go arm init position");
  setArmAngle(0, 0, 0, 0);
}

// Walking
void MainWindow::on_button_init_gyro_clicked(bool check)
{
  qnode_.initGyro();
}

void MainWindow::on_button_walking_start_clicked(bool check)
{
  is_walking_ = true;
  qnode_.setWalkingCommand("start");
}

void MainWindow::on_button_walking_stop_clicked(bool check)
{
  is_walking_ = false;
  qnode_.setWalkingCommand("stop");
}

void MainWindow::on_button_param_refresh_clicked(bool check)
{
  qnode_.refreshWalkingParam();
}

void MainWindow::on_button_param_save_clicked(bool check)
{
  qnode_.setWalkingCommand("save");
}

void MainWindow::on_button_param_apply_clicked(bool check)
{
  applyWalkingParams();
}

/*****************************************************************************
 ** Implemenation [Slots][manually connected]
 *****************************************************************************/
/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView()
{
  ui_.view_logging->scrollToBottom();
}

// User Shortcut
void MainWindow::setUserShortcut()
{
  // Setup a signal mapper to avoid creating custom slots for each tab
  QSignalMapper *_sig_map = new QSignalMapper(this);

  // Setup the shortcut for the first tab : Mode
  QShortcut *_short_tab1 = new QShortcut(QKeySequence("F1"), this);
  connect(_short_tab1, SIGNAL(activated()), _sig_map, SLOT(map()));
  _sig_map->setMapping(_short_tab1, 0);

  // Setup the shortcut for the second tab : Arm Control
  QShortcut *_short_tab2 = new QShortcut(QKeySequence("F2"), this);
  connect(_short_tab2, SIGNAL(activated()), _sig_map, SLOT(map()));
  _sig_map->setMapping(_short_tab2, 1);

  // Setup the shortcut for the fouth tab : Action Control
  QShortcut *_short_tab5 = new QShortcut(QKeySequence("F3"), this);
  connect(_short_tab5, SIGNAL(activated()), _sig_map, SLOT(map()));
  _sig_map->setMapping(_short_tab5, 4);

  // Setup the shortcut for the fouth tab : Walking Control
  QShortcut *_short_tab6 = new QShortcut(QKeySequence("F4"), this);
  connect(_short_tab6, SIGNAL(activated()), _sig_map, SLOT(map()));
  _sig_map->setMapping(_short_tab6, 5);

  // Wire the signal mapper to the tab widget index change slot
  connect(_sig_map, SIGNAL(mapped(int)), ui_.tabWidget_control, SLOT(setCurrentIndex(int)));

  QShortcut *walking_shortcut = new QShortcut(QKeySequence(Qt::Key_Space), this);
  connect(walking_shortcut, SIGNAL(activated()), this, SLOT(walkingCommandShortcut()));
}

// mode control
// it's not used now
void MainWindow::setMode(bool check)
{
  robotis_controller_msgs::JointCtrlModule _control_msg;

  QList<QComboBox *> _combo_children = ui_.widget_mode->findChildren<QComboBox *>();
  for (int ix = 0; ix < _combo_children.length(); ix++)
  {
    std::stringstream _stream;
    std::string _joint;
    int _id;

    int _control_index = _combo_children.at(ix)->currentIndex();
    // if(_control_index == QNodeThor3::Control_None) continue;

    std::string _control_mode = _combo_children.at(ix)->currentText().toStdString();

    if (qnode_.getIDJointNameFromIndex(ix, _id, _joint) == true)
    {
      _stream << "[" << (_id < 10 ? "0" : "") << _id << "] " << _joint << " : " << _control_mode;

      _control_msg.joint_name.push_back(_joint);
      _control_msg.module_name.push_back(_control_mode);
    }
    else
    {
      _stream << "id " << ix << " : " << _control_mode;
    }

    qnode_.log(QNode::Info, _stream.str());
  }

  // no control
  if (_control_msg.joint_name.size() == 0)
    return;

  qnode_.log(QNode::Info, "set mode");

  qnode_.setJointControlMode(_control_msg);
}

void MainWindow::updateCurrentJointMode(std::vector<int> mode)
{
  QList<QComboBox *> _combo_children = ui_.widget_mode->findChildren<QComboBox *>();
  for (int ix = 0; ix < _combo_children.length(); ix++)
  {
    int _control_index = mode.at(ix);
    _combo_children.at(ix)->setCurrentIndex(_control_index);

    if (debug_)
    {
      std::stringstream _stream;
      std::string _joint;
      int _id;

      std::string _control_mode = _combo_children.at(ix)->currentText().toStdString();

      if (qnode_.getIDJointNameFromIndex(ix, _id, _joint) == true)
      {
        _stream << "[" << (_id < 10 ? "0" : "") << _id << "] " << _joint << " : " << _control_mode;
      }
      else
      {
        _stream << "id " << ix << " : " << _control_mode;
      }

      qnode_.log(QNode::Info, _stream.str());
    }
  }

  // set module UI
  updateModuleUI();
}

void MainWindow::updateModuleUI()
{
  if (debug_)
    return;

  for (int index = 0; index < qnode_.getModeSize(); index++)
  {
    std::string _mode = qnode_.getModeName(index);
    if (_mode == "")
      continue;

    std::map<std::string, QList<QWidget *> >::iterator _module_iter = module_ui_table_.find(_mode);
    if (_module_iter == module_ui_table_.end())
      continue;

    bool _is_enable = qnode_.isUsingModule(_mode);

    QList<QWidget *> _list = _module_iter->second;
    for (int ix = 0; ix < _list.size(); ix++)
    {
      _list.at(ix)->setEnabled(_is_enable);
    }
  }

  // refresh walking parameter
  if (qnode_.isUsingModule("walking_module"))
    qnode_.refreshWalkingParam();
}


/*****************************************************************************
** Arm Control
*****************************************************************************/
void MainWindow::updateArmAngles(double r_shoulder_roll, double r_shoulder_pitch, double l_shoulder_roll, double l_shoulder_pitch)
{
  if (ui_.r_shoulder_roll_slider->underMouse() == true)
    return;
  if (ui_.r_shoulder_roll_spinbox->underMouse() == true)
    return;
  if (ui_.r_shoulder_pitch_slider->underMouse() == true)
    return;
  if (ui_.r_shoulder_pitch_spinbox->underMouse() == true)
    return;
  if (ui_.l_shoulder_roll_slider->underMouse() == true)
    return;
  if (ui_.l_shoulder_roll_spinbox->underMouse() == true)
    return;
  if (ui_.l_shoulder_pitch_slider->underMouse() == true)
    return;
  if (ui_.l_shoulder_pitch_spinbox->underMouse() == true)
    return;

  is_updating_ = true;

  ui_.r_shoulder_roll_slider->setValue(r_shoulder_roll * 180.0 / M_PI);
  ui_.r_shoulder_pitch_slider->setValue(r_shoulder_pitch * 180.0 / M_PI);
  ui_.l_shoulder_roll_slider->setValue(l_shoulder_roll * 180.0 / M_PI);
  ui_.l_shoulder_pitch_slider->setValue(l_shoulder_pitch * 180.0 / M_PI);

  is_updating_ = false;
}

void MainWindow::setArmAngle()
{
  if (is_updating_ == true)
    return;
  qnode_.setArmJoint(ui_.r_shoulder_roll_slider->value() * M_PI / 180, 
                     ui_.r_shoulder_pitch_slider->value() * M_PI / 180,
                     ui_.l_shoulder_roll_slider->value() * M_PI / 180,
                     ui_.l_shoulder_pitch_slider->value() * M_PI / 180);
}

void MainWindow::setArmAngle(double r_shoulder_roll, double r_shoulder_pitch, double l_shoulder_roll, double l_shoulder_pitch)
{
  qnode_.setArmJoint(r_shoulder_roll  * M_PI / 180, 
                     r_shoulder_pitch * M_PI / 180,
                     l_shoulder_roll  * M_PI / 180,
                     l_shoulder_pitch * M_PI / 180);
}

/*****************************************************************************
** Walking
*****************************************************************************/
void MainWindow::updateWalkingParams(robotis_engineer_walking_module_msgs::WalkingParam params)
{
  // init pose
  ui_.dSpinBox_init_offset_x->setValue(params.init_x_offset);
  ui_.dSpinBox_init_offset_y->setValue(params.init_y_offset);
  ui_.dSpinBox_init_offset_z->setValue(params.init_z_offset);
  ui_.dSpinBox_init_offset_roll->setValue(params.init_roll_offset * RADIAN2DEGREE);
  ui_.dSpinBox_init_offset_pitch->setValue(params.init_pitch_offset * RADIAN2DEGREE);
  ui_.dSpinBox_init_offset_yaw->setValue(params.init_yaw_offset * RADIAN2DEGREE);
  ui_.dSpinBox_hip_pitch_offset->setValue(params.hip_pitch_offset * RADIAN2DEGREE);

  // time
  ui_.dSpinBox_period_time->setValue(params.period_time * 1000);       // s -> ms
  ui_.dSpinBox_dsp_ratio->setValue(params.dsp_ratio);
  ui_.dSpinBox_step_fb_ratio->setValue(params.step_fb_ratio);

  // walking
  ui_.dSpinBox_x_move_amplitude->setValue(params.x_move_amplitude);
  ui_.dSpinBox_y_move_amplitude->setValue(params.y_move_amplitude);
  ui_.dSpinBox_z_move_amplitude->setValue(params.z_move_amplitude);
  ui_.dSpinBox_y_move_amplitude->setValue(params.angle_move_amplitude);
  ui_.checkBox_move_aim_on->setChecked(params.move_aim_on);
  ui_.checkBox_move_aim_off->setChecked(!params.move_aim_on);
}

void MainWindow::applyWalkingParams()
{
  robotis_engineer_walking_module_msgs::WalkingParam walking_param;

  // init pose
  walking_param.init_x_offset = ui_.dSpinBox_init_offset_x->value();
  walking_param.init_y_offset = ui_.dSpinBox_init_offset_y->value();
  walking_param.init_z_offset = ui_.dSpinBox_init_offset_z->value();
  walking_param.init_roll_offset = ui_.dSpinBox_init_offset_roll->value() * DEGREE2RADIAN;
  walking_param.init_pitch_offset = ui_.dSpinBox_init_offset_pitch->value() * DEGREE2RADIAN;
  walking_param.init_yaw_offset = ui_.dSpinBox_init_offset_yaw->value() * DEGREE2RADIAN;
  walking_param.hip_pitch_offset = ui_.dSpinBox_hip_pitch_offset->value() * DEGREE2RADIAN;

  // time
  walking_param.period_time = ui_.dSpinBox_period_time->value() * 0.001;     // ms -> s
  walking_param.dsp_ratio = ui_.dSpinBox_dsp_ratio->value();
  walking_param.step_fb_ratio = ui_.dSpinBox_step_fb_ratio->value();

  // walking
  walking_param.x_move_amplitude = ui_.dSpinBox_x_move_amplitude->value();
  walking_param.y_move_amplitude = ui_.dSpinBox_y_move_amplitude->value();
  walking_param.z_move_amplitude = ui_.dSpinBox_z_move_amplitude->value();
  walking_param.angle_move_amplitude = ui_.dSpinBox_a_move_amplitude->value() * DEGREE2RADIAN;
  walking_param.move_aim_on = ui_.checkBox_move_aim_on->isChecked();

  qnode_.applyWalkingParam(walking_param);
}

void MainWindow::walkingCommandShortcut()
{
  if (is_walking_ == true)
  {
    is_walking_ = false;
    qnode_.setWalkingCommand("stop");
  }
  else
  {
    is_walking_ = true;
    qnode_.setWalkingCommand("start");
  }
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/
void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(this, tr("About ..."), tr("<h2>ROBOTIS ENGINEER Demo 0.10</h2><p>Copyright ROBOTIS</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/
void MainWindow::initModeUnit()
{
  int number_joint = qnode_.getJointSize();

  // preset button
  QHBoxLayout *preset_layout = new QHBoxLayout;
  QSignalMapper *signalMapper = new QSignalMapper(this);

  // yaml preset
  for (std::map<int, std::string>::iterator module_it = qnode_.module_table_.begin();
      module_it != qnode_.module_table_.end(); ++module_it)
  {
    std::string preset_name = module_it->second;
    QPushButton *preset_button = new QPushButton(tr(preset_name.c_str()));
    if (debug_)
      std::cout << "name : " << preset_name << std::endl;

    preset_layout->addWidget(preset_button);

    signalMapper->setMapping(preset_button, preset_button->text());
    QObject::connect(preset_button, SIGNAL(clicked()), signalMapper, SLOT(map()));
  }

  QObject::connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(setMode(QString)));

  ui_.widget_mode_preset->setLayout(preset_layout);

  // joints
  QGridLayout *grid_layout = new QGridLayout;
  for (int ix = 0; ix < number_joint; ix++)
  {
    std::stringstream label_stream;
    std::string joint_name;
    int joint_id;

    if (qnode_.getIDJointNameFromIndex(ix, joint_id, joint_name) == false)
      continue;

    label_stream << "[" << (joint_id < 10 ? "0" : "") << joint_id << "] " << joint_name;
    QLabel *id_label = new QLabel(tr(label_stream.str().c_str()));

    QStringList module_list;
    for (int index = 0; index < qnode_.getModeSize(); index++)
    {
      std::string module_name = qnode_.getModeName(index);
      if (module_name != "")
        module_list << module_name.c_str();
    }

    QComboBox *module_combo = new QComboBox();
    module_combo->setObjectName(tr(joint_name.c_str()));
    module_combo->addItems(module_list);
    module_combo->setEnabled(false);      // not changable
    int num_row = ix / 2 + 1;
    int num_col = (ix % 2) * 3;
    grid_layout->addWidget(id_label, num_row, num_col, 1, 1);
    grid_layout->addWidget(module_combo, num_row, num_col + 1, 1, 2);
  }

  // get/set buttons
  QPushButton *get_mode_button = new QPushButton(tr("Get Mode"));
  grid_layout->addWidget(get_mode_button, (number_joint / 2) + 2, 0, 1, 3);
  QObject::connect(get_mode_button, SIGNAL(clicked(bool)), &qnode_, SLOT(getJointControlMode()));

  ui_.widget_mode->setLayout(grid_layout);

  // make module widget table
  for (int index = 0; index < qnode_.getModeSize(); index++)
  {
    std::string module_name = qnode_.getModeName(index);
    if (module_name == "")
      continue;
    std::string module_reg = "*_" + module_name;

    QRegExp reg_exp(QRegExp(tr(module_reg.c_str())));
    reg_exp.setPatternSyntax(QRegExp::Wildcard);

    QList<QWidget *> widget_list = ui_.centralwidget->findChildren<QWidget *>(reg_exp);
    module_ui_table_[module_name] = widget_list;

    if (debug_)
      std::cout << "Module widget : " << module_name << " [" << widget_list.size() << "]" << std::endl;
  }

  // make motion tab
  if (qnode_.getModeIndex("action_module") != -1)
    initMotionUnit();
}

/*****************************************************************************
** Motion Control
*****************************************************************************/
void MainWindow::initMotionUnit()
{
  // preset button
  QGridLayout *motion_layout = new QGridLayout;
  QSignalMapper *signal_mapper = new QSignalMapper(this);

  // yaml preset
  int index = 0;
  for (std::map<int, std::string>::iterator motion_it = qnode_.motion_table_.begin();
      motion_it != qnode_.motion_table_.end(); ++motion_it)
  {
    int motion_index = motion_it->first;
    std::string motion_name = motion_it->second;
    QString q_motion_name = QString::fromStdString(motion_name);
    QPushButton *motion_button = new QPushButton(q_motion_name);

    int button_size = (motion_index < 0) ? 2 : 1;
    int num_row = index / 4;
    int num_col = index % 4;
    motion_layout->addWidget(motion_button, num_row, num_col, 1, button_size);

    //hotkey
    std::map<int, int>::iterator shortcut_it = qnode_.motion_shortcut_table_.find(motion_index);
    if (shortcut_it != qnode_.motion_shortcut_table_.end())
      motion_button->setShortcut(QKeySequence(shortcut_it->second));

    signal_mapper->setMapping(motion_button, motion_index);
    QObject::connect(motion_button, SIGNAL(clicked()), signal_mapper, SLOT(map()));

    index += button_size;
  }

  int num_row = index / 4;
  num_row = (index % 4 == 0) ? num_row : num_row + 1;
  QSpacerItem *vertical_spacer = new QSpacerItem(20, 400, QSizePolicy::Minimum, QSizePolicy::Expanding);
  motion_layout->addItem(vertical_spacer, num_row, 0, 1, 4);

  QObject::connect(signal_mapper, SIGNAL(mapped(int)), &qnode_, SLOT(playMotion(int)));

  ui_.scroll_widget_motion->setLayout(motion_layout);
}

void MainWindow::setMode(QString mode_name)
{
  qnode_.setControlMode(mode_name.toStdString());
}

void MainWindow::readSettings()
{
  QSettings settings("Qt-Ros Package", "robotis_engineer_gui_demo");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::writeSettings()
{
  QSettings settings("Qt-Ros Package", "robotis_engineer_gui_demo");
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  writeSettings();
  QMainWindow::closeEvent(event);
}

}  // namespace robotis_engineer

