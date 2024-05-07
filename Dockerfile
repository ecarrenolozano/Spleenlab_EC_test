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

#-- Installing some utilities for prompting information and live code editing on the docker container
RUN apt-get update\
    && apt-get install -y build-essential \
    && apt-get install -y dialog apt-utils nano

#-----------------------------      ROS INSTALLATION        ---------------------
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
    && echo "Step 3...SUCCESSFUL"

# Install ROS 2 packages
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y ros-humble-ros-base \
    && apt-get install -y python3-rosdep \
    && apt-get install -y python3-colcon-common-extensions \
    && echo "Step 4...SUCCESSFUL"

#---------------------------        WORKSPACE PREPARATION        -------------------------
#-- Choosing /bin/bash as a default shell
SHELL [ "/bin/bash" ]

# Create ROS Workspace
WORKDIR /home/ROS_WS/src

# Create folder for all file sources and non-ROS executables
#RUN ["mkdir", "/home/sources/", "/home/executables/"]

# Copy files from host machine to docker image
COPY --chmod=777 ./executables/ /home/executables/
COPY ./sources/ /home/sources/
COPY ./CMakeLists.txt ./package.xml /home/
      
ENV PACKAGE_NAME="cpp_pubsub"
RUN ["/home/executables/create_project.sh"]

ENTRYPOINT ["/home/executables/entrypoint_script.sh"]