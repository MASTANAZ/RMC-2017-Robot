#!/bin/bash
clear
export ROS_IP=192.168.1.4
export ROS_MASTER_URI=http://192.168.1.4:11311
export ROS_NAMESPACE=phobos
source $HOME/om_ws/devel/setup.bash
roslaunch om17 om17.launch
