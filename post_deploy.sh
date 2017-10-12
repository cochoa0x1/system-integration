#!/usr/bin/env bash
cd ros
catkin_make
source devel/setup.sh
rosnode kill -a
roslaunch launch/styx.launch
