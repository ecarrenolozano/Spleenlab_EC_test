#!/bin/bash

PACKAGE_NAME="cpp_pubsub"

#-- Source ROS setup file
. /opt/ros/humble/setup.sh

#-- Create ROS package for the project
ros2 pkg create --build-type ament_cmake --license Apache-2.0 $PACKAGE_NAME

#-- Move all source files to src folder into the package folder
mv /home/*.cpp /home/ROS_WS/src/$PACKAGE_NAME/src/

mv /home/package.xml /home/CMakeLists.txt /home/ROS_WS/src/$PACKAGE_NAME

cd /home/ROS_WS

rosdep init
rosdep update

rosdep install -i --from-path src --rosdistro humble -y

colcon build --packages-select $PACKAGE_NAME

#. install/setup.bash



