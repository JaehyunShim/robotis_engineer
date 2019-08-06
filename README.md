### ROBOTIS ENGINEER

#### 1. Intro

#### 2. How to Use

#### 3. Install ROS packages for ROBOTIS Engineer
```sh
(Move to your catkin workspace)
$ cd ~/catkin_ws/src/

(Download packages)
$ git clone https://github.com/rjshim/robotis_engineer.git
$ git clone https://github.com/rjshim/robotis_engineer_msgs.git
$ git clone https://github.com/rjshim/robotis_engineer_simulations.git
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```

#### 4. Execute ROS packages for ROBOTIS Engineer
- Press the left-side button until the light turns red. Then, press the right-side button.
- Run below in the terminal window.

```sh
(Manager)
$ sudo bash
$ roslaunch robotis_engineer_manager robotis_engineer_manager.launch

(Action Editor)
$ roslaunch robotis_engineer_action_editor robotis_engineer_action_editor.launch

(GUI)
$ roslaunch robotis_engineer_control_gui robotis_engineer_control_gui.launch

(Rviz)
$ roslaunch robotis_engineer_description robotis_engineer_rviz.launch

(Gazebo)
$ roslaunch robotis_engineer_gazebo robotis_engineer_gazebo.launch
```

#### Reference
- [robotis_op3](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3)
- [robotis_op3_msgs](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-msgs)
- [robotis_op3_tools](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-Tools)
