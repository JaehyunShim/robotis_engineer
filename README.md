# ROBOTIS ENGINEER

## Usage
- Needed Items: [Robotis-Engineer Max-E1](http://www.robotis.com/shop/item.php?it_id=901-0153-100) x 1

### Install ROS Kinetic
```sh
$ sudo apt-get update && sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/jrshim/robotis_engineer /master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh
```

### Install ROS packages and Build
```sh
(Move to your catkin workspace)
$ cd ~/catkin_ws/src/

(Download packages)
$ git clone https://github.com/rjshim/robotis_engineer.git
$ git clone https://github.com/rjshim/robotis_engineer_msgs.git
$ git clone https://github.com/rjshim/robotis_engineer_simulations.git

(Install ROS packages)
$ sudo apt-get install ros-kinetic-robotis-framework*

(Build)
$ cd ~/catkin_ws && catkin_make
```

### Execute ROS packages
- Press the left-side button until the light turns red. Then, press the right-side button.
- Run below in the terminal window.

```sh
(Manager)
$ sudo bash
$ roslaunch robotis_engineer_manager robotis_engineer_manager.launch

(GUI)
$ roslaunch robotis_engineer_control_gui robotis_engineer_control_gui.launch

(Rviz)
$ roslaunch robotis_engineer_description robotis_engineer_rviz.launch

(Gazebo)
$ roslaunch robotis_engineer_gazebo robotis_engineer_gazebo.launch
```

If you want to make more actions, run below
```sh
(Action Editor)
$ sudo bash
$ roslaunch robotis_engineer_action_editor robotis_engineer_action_editor.launch
```

### Reference
- [robotis_engineer_msgs](https://github.com/rjshim/robotis_engineer_msgs)
- [robotis_engineer_simulations](https://github.com/rjshim/robotis_engineer_simulations)
- [robotis_op3](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3)
- [robotis_op3_msgs](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-msgs)
- [robotis_op3_tools](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-Tools)
