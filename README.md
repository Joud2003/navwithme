# Autonomous Hazard-Aware Mobile Robot

ROS2-based autonomous mobile robot capable of environment exploration, mapping, localisation, hazard detection, and autonomous waypoint navigation using Gazebo simulation.

## Overview

The aim of this project is to develop an autonomous mobile robot that explores an environment, detects potentially dangerous items from a user-defined list using vision-based perception, localises these items with confidence filtering, and then generates navigation behaviour that allows the robot to guide an operator to the detected hazards for inspection or intervention.

The system integrates autonomous exploration, real-time mapping, perception-driven hazard identification, and Nav2-based navigation to hazardous locations while maintaining robust localisation throughout operation.

During operation, the robot performs structured exploration using a lawn-mower coverage pattern while simultaneously building an occupancy grid map and localising itself using SLAM Toolbox.

Potential objects are detected using LiDAR-based cues, then confirmed using camera images processed through a YOLO-based classification model. Valid hazard detections are filtered, clustered spatially, and stored in map coordinates with associated confidence scores.

After exploration, the robot returns to its starting position and executes an intervention phase where it autonomously navigates to detected hazard locations using Nav2.

## Features

* ROS2-based modular architecture
* TurtleBot3 simulation in Gazebo
* PID waypoint control
* Lawn-mower (boustrophedon) coverage navigation
* SLAM Toolbox mapping and localisation
* Occupancy grid generation
* LiDAR-based object detection
* Detection clustering and filtering
* YOLO-based object classification
* Nav2 waypoint navigation and path planning
* Localisation and trajectory validation tools

## System Architecture

Disclaimer: generated using AI

<img width="1536" height="1024" alt="System Architecture" src="https://github.com/user-attachments/assets/0c448a0b-9aa6-484b-8824-485d2dafe424" />

## Workflow

1. Explore environment using lawn-mower navigation.
2. Build occupancy grid map using SLAM Toolbox.
3. Detect candidate objects using LiDAR.
4. Capture camera images when objects are detected.
5. Classify objects using YOLO.
6. Filter duplicate detections using spatial clustering.
7. Store hazard locations in map coordinates.
8. Return to starting position.
9. Navigate to detected hazards using Nav2.

## Technologies

* ROS2 Humble
* Python
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
nav_with_me/
├── launch/
├── constants/
├── robot_navigation/
├── pid_controller/
├── img_object_detection/
├── motion_controller/
├── robot_data/
└── test/
```

## Future Improvements

* Real-world robot deployment
* Improved object detection datasets
* Semantic mapping
* Multi-room exploration
* Dynamic obstacle handling

## Author

Joud Salhi

Robotics, Autonomous Systems, ROS2 and Software Engineering

