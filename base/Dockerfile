#---------------------------------------------------------
#----               Docke4rfile ROS2 Humble           ----
#       Author: Edwin Carreño                         ----
#  Last Update: 2024.04.09                            ----
#---------------------------------------------------------

# Use Ubuntu 22.04 as the base image.
FROM ubuntu:22.04

# Environment variable (building phase) to avoid console interaction while building the image.
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y dialog apt-utils

# Set locale  -- DONE
RUN apt-get update && apt-get install locales -y \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && echo "export LANG=en_US.UTF-8" >> ~/.bashrc \
    && echo "Step 1...SUCCESSFUL"

# Setup sources --DONE
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
    && apt-get install python3-rosdep \
    && apt rosdep init \
    && rosdep update \
    && apt install -y python3-colcon-common-extensions \ 
    && echo "Step 4...SUCCESSFUL"



# Entrypoint script
#-- sourcing the setup script
#COPY entrypoint_script.sh /entrypoint_script.sh
#COPY test.sh /test.sh

#ENTRYPOINT [ "/bin/bash", "-c","./entrypoint_script.sh" ]







