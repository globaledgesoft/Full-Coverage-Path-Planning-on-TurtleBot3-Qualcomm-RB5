
# Integration of Full Coverage Path Planning Packages with TurtleBot3 and Qualcomm RB5

This project focuses on the seamless integration of full coverage path planning (FCPP) packages with the TurtleBot3 robotic platform, leveraging the computational power of the Qualcomm RB5 platform. The goal is to enable autonomous navigation that ensures comprehensive area coverage, making it ideal for tasks such as environment exploration, cleaning, and inspection.

### Use Cases
-   **Environment Exploration:** Navigate and map unknown areas autonomously.

-   **Cleaning Robots:** Ensure complete coverage of a floor or space, similar to robotic vacuum cleaners.

-   **Inspection Tasks:** Conduct thorough inspections of environments, ensuring no area is missed.

-   **Agricultural Applications:** Cover entire fields for tasks like crop monitoring or pesticide application.

-   **Security Patrols:** Automatically patrol and monitor large areas without leaving any gaps.

## Setup guide

####  Requirements
- Python 3
- ROS Melodic Installation on RB5 & Host Machine (ubuntu 18) [Installation Guide](http://wiki.ros.org/melodic/Installation/Ubuntu) 
- Install TurtleBot3 Packages on Host Machine [Installation Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#install-turtlebot3-packages)

    
#### Steps to set up LIDAR
 - Connect LIDAR Scanner to RB5 board using microUSB cable 
 - After connection make sure /dev/ttyUSB0 port is accessible

#### Add below lines in ~/.bashrc of RB5

```sh
# APPEND AT THE END OF ~/.bashrc
export ROS_MASTER_URI=http://<HOST_MACHINE_IP>:11311
export ROS_HOSTNAME=<RB5_IP>
export TURTLEBOT3_MODEL=burger
source /opt/ros/melodic/setup.bash

``` 
#### Add below lines in ~/.bashrc of Host Machine

```sh
# APPEND AT THE END OF ~/.bashrc
export ROS_MASTER_URI=http://<HOST_MACHINE_IP>:11311
export ROS_HOSTNAME=<HOST_MACHINE_IP>
export TURTLEBOT3_MODEL=burger
source /opt/ros/melodic/setup.bash
``` 

### Prepare ROS workspace

First, initialize your ROS workspace according to the instructions in the [ROS tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment). Then, clone the Git repository into your workspace. 

## Workspace in Host Machine

To set up the ROS workspace on the host machine, follow these steps:

Initialize the ROS Workspace:
Create a directory for your ROS workspace, initialize it, and set up the necessary environment variables.
 
```sh
$ mkdir -p ~/FCPP/src 
$ cd ~/FCPP/src 
```
Copy the packages in the assets folder to the ~/FCPP/src directory. 

```sh
$ cp -r assets/TURTLEBOT3_FULL_COVERAGE_PATH_PLANNER ~/FCPP/src/
```
Download full_coverage_path_planner and tracking_pid from git and place it in ~/FCPP/src

```sh
    $ cd ~/FCPP/src
    $ git clone https://github.com/nobleo/full_coverage_path_planner
    $ git clone https://github.com/nobleo/tracking_pid.git
```

Install all the dependencies through rosdep, 
```sh
$  cd ~/FCPP
$ rosdep install --from-paths src --ignore-src --rosdistro melodic -yr 
```
Source ROS 1 and build,

```sh
$ cd ~/FCPP
$ catkin_make 
$ source ~/FCPP/devel/setup.bash 
```

Run the command given below to update the ~/.bashrc 
```sh
$ echo source ~/FCPP/devel/setup.bash >> ~/.bashrc 
```

## Execution instruction

-   On Host System, Source the melodic and run roscore
```sh
$ roscore
```
- On RB5 shell, launch bring up command 

```sh
sh4.4 roslaunch turtlebot3_bringup turtlebot3_robot.launch 
```

- On host run robot_state_publisher node

```sh
$ roslaunch robot_gazebo test.launch
```
- Launching Full Coverage Path Planning for TurtleBot3 on Host System

```sh
$  roslaunch robot_navigation rb5_FCPP.launch 
```

## Demo 

![Animation](./image/output.gif)

### Out of scope
-   Dynamic Obstacle Avoidance:

    -   Enhancing path planning algorithms to better handle dynamic obstacles and environmental changes during coverage tasks.
        
    -   Incorporating real-time sensor data to adapt coverage paths in response to unexpected changes.

### References
- https://github.com/nobleo/full_coverage_path_planner
- https://github.com/nobleo/tracking_pid
