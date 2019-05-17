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
(1. manager)
$ roslaunch max_manager max_manager.launch
$ roslaunch max_manager max_manager_beta.launch (when using the robot without legs)

(2. Action Editor)
$ roslaunch max_action_editor max_action_editor.launch 

(3. GUI)
$ roslaunch max_gui_demo max_gui_demo.launch 

(4. Tester)
$ rosrun max_tester max_tester
```

## Reference
1) [ROBOTIS-OP3](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3)
2) [ROBOTIS-OP3-msgs](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-msgs)
3) [ROBOTIS-OP3-Tools](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-Tools)
