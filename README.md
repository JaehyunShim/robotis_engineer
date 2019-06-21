### ROBOTIS ENGINEER

#### 1. Intro

#### 2. How to Use

#### 3. Install ROS packages for ROBOTIS-MAX
```sh
(Move to your catkin workspace)
$ cd ~/catkin_ws/src/

(Download packages)
$ git clone https://github.com/RyanJaehyunShim/robotis_max.git
$ git clone https://github.com/RyanJaehyunShim/robotis_max_msgs.git
$ git clone https://github.com/RyanJaehyunShim/robotis_max_simulations.git
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```

#### 4. Execute ROS packages for ROBOTIS-MAX\
- Press left the left-side button until the light turns red. Then,press the right-side button.
- Run below in the terminal window.
```sh
$ sudo bash

(Manager)
$ roslaunch robotis_engineer_manager robotis_engineer_manager.launch
or
$ roslaunch robotis_engineer_manager robotis_engineer_manager_beta.launch (if using the no-leg-version)

(Action Editor)
$ roslaunch robotis_engineer_action_editor robotis_engineer_action_editor.launch

(GUI)
$ roslaunch robotis_engineer_control_gui robotis_engineer_control_gui.launch

(Rviz)
$ roslaunch robotis_engineer_description robotis_engineer_rviz.launch

(Gazebo)
$ roslaunch robotis_engineer_gazebo robotis_engineer_gazebo.launch
```
#### 5. Contribution
- Metapacakge added.
- Motor info in manager has been modified according to the DXL model for ROBOTIS-MAX.
- Manager-beta has been added for the model without legs.
- Geometric kdl for robot legs has been implemented instead of using [orocos kdl](http://www.orocos.org/kdl).
- New motions for ROBOTIS-MAX are added. (TODO)
- Description and Gazebo packages have been modified. (TODO)

#### 6. Future work
- kinematics_module역할??? 왜 kdl 참조>..?   humanoid_kdl, kinematics_dynamics...
- opencr_module??
- balance control???

#### Reference
- [robotis_op3](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3)
- [robotis_op3_msgs](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-msgs)
- [robotis_op3_tools](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-Tools)
