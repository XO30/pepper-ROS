#!/bin/bash

# initialisation of the ROS environment
export PATH=$PATH:$HOME/.local/bin$
export PYTHONPATH=/pynaoqi-python2.7-pynaoqi-python2.7-2.8.7.4-linux64-20210819_141148/lib/python2.7/site-packages:${PYTHONPATH}
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

echo "Starting ROS..."

# start of the ROS launch file and pipe the output to a log file
roslaunch pepper_launch pepper_launch.launch 2>&1 | tee /root/ros_launch_log.txt

echo "ROS launch started."

# loop to keep the container running
while true; do
    sleep 1
done
