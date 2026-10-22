# Autonomous Hazard-Aware Mobile Robot: NavWithMe

ROS 2 project with a Python perception node and a C++ motion-control node for TurtleBot3 simulation, environment exploration, mapping, and hazard detection.

## Overview

The aim of this project is to develop an autonomous mobile robot that explores an environment, detects potentially dangerous items from a user-defined list using vision-based perception, localises these items with confidence filtering, and then generates navigation behaviour that allows the robot to guide an operator to the detected hazards for inspection or intervention.
The project explores a two-node architecture: Python handles sensor-based perception, while C++ controls the robot's exploration motion.

The C++ node reads `/scan` and `/odom`, runs a PD-based coverage controller, and publishes velocity commands on `/cmd_vel`. The Python node processes LiDAR and camera data, runs YOLO classification, and listens for the C++ node's `/exploration_completed` status event.

During exploration, SLAM Toolbox can build an occupancy grid while the C++ controller follows a lawn-mower pattern. The Python node currently logs the completion event; it does not yet pass detected-object data to the C++ controller or trigger the Python Nav2 return/waypoint behavior from that event.

Potential objects are detected using LiDAR-based cues, then confirmed using camera images processed through a YOLO-based classification model. Valid hazard detections are filtered, clustered spatially, and stored in map coordinates with associated confidence scores.

After exploration, the robot returns to its starting position and executes an intervention phase where it autonomously navigates to detected hazard locations using Nav2.

## Features

* ROS2-based modular architecture
* TurtleBot3 simulation in Gazebo
* C++ PD-based lawn-mower coverage control
* Python LiDAR and camera perception
* YOLO-based object classification and detection filtering
* ROS topic notification when C++ exploration completes
* SLAM Toolbox mapping and localisation
* Occupancy grid generation
* Nav2 action-client and waypoint components
* Localisation and trajectory validation tools

## System Architecture

Disclaimer: generated using AI

<img width="1536" height="1024" alt="System Architecture" src="https://github.com/user-attachments/assets/0c448a0b-9aa6-484b-8824-485d2dafe424" />

## Workflow

1. Launch the simulator, Python perception node, and C++ motion-control node.
2. The C++ node uses LiDAR and odometry to execute the configured coverage pattern and publishes commands on `/cmd_vel`.
3. The Python node processes LiDAR cues and camera images, classifies candidate objects, and records detections.
4. When the C++ controller completes exploration, it publishes `true` on `/exploration_completed`; Python logs the event and initiates return home using Nav2.
5. Upon returning to the start position, Nav2 is used to renavigate through detected hazards.

## Technologies

* ROS2 Humble
* Python
* C++
* Gazebo Classic
* TurtleBot3
* Nav2
* SLAM Toolbox
* OpenCV
* YOLOv8
* NumPy

## Validation

Localisation performance was evaluated by comparing estimated robot poses against simulation ground truth.

Example results:

* Sparse environment:

  * Mean position error: 0.002 m
  * Maximum position error: 0.012 m

* Rich environment:

  * Mean position error: 0.049 m
  * Maximum position error: 0.069 m

## Repository Structure

```text
.
├── nav_with_me/
│   ├── launch/
│   │   ├── custom_turtlebot3_world.launch.py
│   │   └── run_both_nodes.launch.py
│   ├── nav_with_me/
│   │   ├── constants.py
│   │   ├── img_object_detection.py
│   │   ├── motion_controller.py
│   │   ├── object_detection.py
│   │   ├── pid_controller.py
│   │   └── robot_navigation.py
│   ├── test/
│   ├── GAZEBO_TESTING_GUIDE.md
│   ├── my_world.world
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
├── robot_controller_cpp/
│   ├── include/robot_controller_cpp/
│   │   ├── motion_controller.hpp
│   │   └── pid_controller.hpp
│   ├── src/
│   │   ├── motion-controller/
│   │   │   ├── motion_controller.cpp
│   │   │   └── pid_controller.cpp
│   │   └── robot_controller.cpp
│   ├── CMakeLists.txt
│   └── package.xml
├── robot_data/
├── yolov8n.pt
├── yolov9c.pt
└── README.md
```

## Future Improvements

* Real-world robot deployment
* Improved object detection datasets
* Semantic mapping
* Multi-room exploration
* Dynamic obstacle handling

The architecture supports map-referenced hazard detection through integration of YOLO detections with robot localisation data. Due to limitations of the simulation environment and the absence of suitable Gazebo object models for the target hazard classes, end-to-end validation of the object-detection pipeline was not fully completed. However, the perception, localisation, mapping, and navigation components were developed and integrated independently.

# Demo:

Lawn-mower coverage pattern:

https://github.com/user-attachments/assets/1029e76d-9c00-46e3-8356-38bb72ce4210

Detected objects SLAM Map:

<img width="425" height="434" alt="detected objects vs map" src="https://github.com/user-attachments/assets/26987a86-0a6c-42bc-9b95-db55fa10dac4" />

Waypoint Accuracy Validation:

<img width="352" height="278" alt="trajectory vs estimated" src="https://github.com/user-attachments/assets/8b4d6c8c-3f05-44ac-a201-7fc22e5f966a" />

Front wall detection and stopping:

https://github.com/user-attachments/assets/47f3c07c-85de-4b2b-b749-4bcda1794aff

## Author

Joud Salhi

Robotics, Autonomous Systems, ROS2 and Software Engineering

