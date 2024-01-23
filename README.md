# TurtleBot Autonomous Navigation

## Project Overview

This repository contains the code for a ROS project that focuses on various aspects of robotics development. The project involves creating a ROS package, writing ROS nodes, publishers, and subscribers, broadcasting and listening to frames, using parameters, and implementing an action client to navigate a robot through multiple poses.

## Learning Outcomes and Skills Developed

- Creating a ROS package.
- Writing ROS nodes, publishers, and subscribers.
- Broadcasting and listening to frames.
- Using parameters.
- Writing an action client to navigate through multiple poses.

## Setup and Installation

To set up and install the required dependencies for this project, follow the steps below:

1. Uninstall the existing OpenCV packages:
    ```bash
    pip3 uninstall opencv-python
    pip3 uninstall opencv-contrib-python
    ```

2. Install the latest version of OpenCV (opencv-contrib-python):
    ```bash
    pip3 install opencv-contrib-python
    ```

3. Clone the repository and install ROS packages:
    ```bash
    cd
    mkdir -p ~/final_ws/src
    cd final_ws
    git clone https://github.com/piyush-g0enka/enpm809Y_fall2023.git -b final src
    rosdep install --from-paths src -y --ignore-src
    colcon build
    ```

4. Create your package for this assignment using your group name as the package name:
    ```bash
    ros2 pkg create group1_final ...
    ```

5. Move the `waypoint_params.yaml` file to the `config` folder in your package.

6. Make adjustments in your `.bashrc` or `.zshrc` file for the new workspace.

## Task Description

### 7.1 Start the Simulation

Run the following command to start the simulation:
```bash
ros2 launch final_project final_project.launch.py
```

This will launch the Gazebo environment with a Turtlebot, logical cameras, and parts.

### 7.2 Read the ArUco Marker

Reuse the code from RWA3 to parse messages published to `/aruco_markers`. Stop the subscription once the marker ID is received.

### 7.3 Retrieve Parameters

Retrieve parameters associated with the ArUco marker ID from `waypoint_params.yaml`. Map parameters to the detected marker ID.

### 7.4 Find Parts

Compute part poses in the map frame by subscribing to each camera topic.

### 7.5 Navigate Through Poses

Write an action client to make the robot navigate through each pose in the correct order. Use the action server `/navigate_through_poses`.

Note: Initialize the robot using programming and use the topic `/amcl_pose` to retrieve the pose during navigation.

For more details, refer to the project documentation and the provided code.

