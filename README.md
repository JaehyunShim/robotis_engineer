# ROBOTIS-MAX
test repository for git use


## Install ROS Kinetic
```sh
$ sudo apt-get update && sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/RyanJaehyunShim/git_test/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh
```

## Install ROS packages for OpenManipulator
```sh
(Move to your catkin workspace)
$ cd ~/catkin_ws/src/
$ rm .rosinstall
$ wstool init

(for OpenManipulator)
$ wstool merge https://raw.githubusercontent.com/RyanJaehyunShim/git_test/master/.openmanipulator.rosinstall 

(run git clone the selected ROS packages)
$ wstool update -j4

(Install ROS packages that depend on the package you selected)
$ rosdep install --from-paths ~/catkin_ws/src --ignore-src -r -y
or
$ rosdep install --from-paths ~/catkin_ws/src --ignore-src -r -y --os=ubuntu:xenial (if use the Linux Mint)
```

