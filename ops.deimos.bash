#!/bin/bash
clear
export ROS_IP=192.168.1.11
export ROS_MASTER_URI=http://192.168.1.11:11311
export ROS_NAMESPACE=deimos
source $HOME/om_ws/devel/setup.bash
roslaunch om17 om17.launch
