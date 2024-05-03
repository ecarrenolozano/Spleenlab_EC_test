#!/bin/bash

#-- Source ROS setup file
. /opt/ros/humble/setup.sh

#-- Create ROS package for the project
ros2 pkg create --build-type ament_cmake --license Apache-2.0 $PACKAGE_NAME

#-- Move all source files to src folder into the package folder
mv /home/sources/minimal_publisher.cpp /home/ROS_WS/src/$PACKAGE_NAME/src/
mv /home/sources/minimal_subscriber.cpp /home/ROS_WS/src/$PACKAGE_NAME/src/

mv /home/package.xml /home/CMakeLists.txt /home/ROS_WS/src/$PACKAGE_NAME

cd /home/ROS_WS

rosdep init
rosdep update

rosdep install -i --from-path src --rosdistro humble -y

colcon build --packages-select $PACKAGE_NAME

# Compile code for the intensive task
g++ -pthread /home/sources/intensive_task.cpp -o /home/executables/intensive
chmod +x /home/executables/intensive