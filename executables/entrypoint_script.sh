#!/bin/bash

COMMAND=$@

SETUP_ROS2="/opt/ros/humble/setup.bash"
SETUP_APP="/home/ROS_WS/install/setup.bash"

. /opt/ros/humble/setup.bash
. /home/ROS_WS/install/setup.bash

# The bash session receive the command to execute in the container
$COMMAND