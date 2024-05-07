# CPU Monitor in ROS2 docker

## Tasks
1. Write a Dockerfile in which you install ROS2 Humble. Use Ubuntu 22.04 as the base image for this, since the ROS packages are already precompiled for this.  (https://docs.ros.org/en/humble/Installation.html)

2. write a ROS Node, which writes the current CPU load in 5 seconds intervals as relative value into a log file cpu_load.log. You need to implement the ROS Node in C++ (https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html). To do this, extend your Dockerfile so that the node is automatically started in the Dockerfile.

3. Create a docker-compose.yaml to automatically start and stop your Docker container with all necessary resources.

4. Write a shell script that reads the cpu_load.log file and prints new messages in the file on the commandline. The shell script should run outside the Docker container.

5. Write a shell script that loads the CPU sufficiently to test your pipeline.

## Instructions

1. Open a Linux or Powershell terminal and clone this repository by using:

    ```shell
    $ git clone https://github.com/ecarrenolozano/Spleenlab_EC_test.git
    ```

2. Navigate into the repo directory:

    ```shell
    $ cd Spleenlab_EC_test
    ```

3. Create a shared directory in the host machine to store the `cpu_load.log` written by the subscriber (detailed later):
   
   ```shell
    $ mkdir -p /home/$USERNAME/logs_ros_docker/
   ```

4. Three containers will be created:
    * **ros2_pub**: publishes the CPU load information into a topic.
    * **ros2_sub**: reads the information published by ros2_pub, at the same time it writes the information to a file `cpu_load.log` in this container.
    * **intensive_app**: it execute an application that demands processor's capacity (all processors in the machine - 2). 
  
    All of this can be made automatically by running:
    ```shell
    $ docker compose up
    ```
5. Open a **new** terminal, navigate to the repository folder, and run the following command to read the `cpu_load.log` file:
    ```shell
    $ chmod +x monitor_script.sh
    ```
    ```shell
    $ ./monitor_script.sh
    ```
    **At this point you will see the CPU load at specific timestamps. The CPU load should be around 90%.**

6. Open a **new** terminal, navigate to the repository folder, and stop the container the intensive task. At this point you will see how the CPU load now is below than 10%:
   
   ```shell
    $ docker compose stop intensive_app
    ```

### That all!
  