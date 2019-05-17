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
or if you use a without-leg-version robot
$ roslaunch max_manager max_manager_beta.launch 

(Action Editor)
$ roslaunch max_action_editor max_action_editor.launch 

(GUI)
$ roslaunch max_gui_demo max_gui_demo.launch 

(Tester)
$ rosrun max_tester max_tester
```
## Contribution

### From Me...
1. 모터 종류 변환
2. Kinematics를 KDL package가 아닌 기하학적으로 푼 ~파일로 변환
3. manger_beta 추가 (without legs)

### Looking for Contributiors
1. kinematics_module역할??? 왜 kdl 참조>..?   humanoid_kdl, kinematics_dynamics...
2. opencr_module??
3. balance control???

## Reference
1) [ROBOTIS-OP3](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3)
2) [ROBOTIS-OP3-msgs](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-msgs)
3) [ROBOTIS-OP3-Tools](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-Tools)
