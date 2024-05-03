#   Name: Dockerfile
#
#   Description: Dockerfile to build ROS Humble image using Ubuntu 22.04
#
#   Company: Spleenlab
#
#   Author: Edwin Carreño
#   Last update: 03.05.2024
#

# Use Ubuntu 22.04 as the base image
FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y dialog apt-utils

# Set locale
RUN apt-get update && apt-get install locales -y \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8 \
    && echo "Step 1...SUCCESSFUL"

# Setup sources
RUN apt-get install software-properties-common -y \
    && add-apt-repository universe -y \
    && echo "Step 2...SUCCESSFUL"

#-- add ROS 2 GPG key
RUN apt-get update \
    && apt-get install -y curl \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \
    $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get install -y nano \
    && echo "Step 3...SUCCESSFUL"

# Install ROS 2 packages
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y ros-humble-desktop \
    && apt-get install -y python3-rosdep \
    && apt-get install -y python3-colcon-common-extensions

# Environment setup
#-- sourcing the setup script
SHELL [ "/bin/bash" ]
ENV ROS_SETUP_FILE /opt/ros/humble/setup.sh

#Create ROS Workspace
WORKDIR /home/ROS_WS/src

COPY ./create_project.sh /home/
COPY ./sources/* /home/
COPY ./package.xml ./CMakeLists.txt /home/

RUN ["chmod", "+x",  "/home/create_project.sh"]
RUN ["sh","/home/create_project.sh"]