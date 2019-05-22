# ROBOTIS-MAX

## Intro

## How to Use

### Install ROS Kinetic
```sh
$ sudo apt-get update && sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/RyanJaehyunShim/git_test/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh
```
### Install ROS packages for ROBOTIS-MAX
```sh
(Move to your catkin workspace)
$ cd ~/catkin_ws/src/

(Download packages)
$ git clone https://github.com/RyanJaehyunShim/robotis_max.git
$ git clone https://github.com/RyanJaehyunShim/robotis_max_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```

### Execute ROS packages for ROBOTIS-MAX
```sh
(Manager)
$ roslaunch max_manager max_manager.launch
or
$ roslaunch max_manager max_manager_beta.launch (if using a without-leg-version)

(Action Editor)
$ roslaunch max_action_editor max_action_editor.launch 

(GUI)
$ roslaunch max_control_gui max_control_gui.launch 

(Tester)
$ rosrun max_tester max_tester
```
## Contribution

### From Me
- Motor info in manager has been modified according to the DXL model for ROBOTIS-MAX.
- Manager-beta has been added for the model without legs.
- Geometric kdl for robot legs has been implemented instead of using [orocos kdl](http://www.orocos.org/kdl)

### Need future contributions to
- kinematics_module역할??? 왜 kdl 참조>..?   humanoid_kdl, kinematics_dynamics...
- opencr_module??
- balance control???

## Reference
- [robotis_op3](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3)
- [robotis_op3_msgs](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-msgs)
- [robotis_op3_tools](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-Tools)
