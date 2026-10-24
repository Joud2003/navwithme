# Autonomous Hazard-Aware Mobile Robot: NavWithMe

ROS 2 project with Python perception and mission coordination plus a C++ motion-control node for TurtleBot3 simulation, environment exploration, mapping, and hazard-detection experiments.

## Overview

The project explores an environment with a lawn-mower coverage controller, classifies candidate objects from camera images, and tests navigation to the resulting map-referenced waypoints. Python handles perception and mission coordination; C++ handles low-level exploration motion.

The C++ node reads `/scan` and `/odom`, runs a PD-based coverage controller, and publishes velocity commands on `/cmd_vel`. It waits for `/perception_ready` before moving and publishes `/exploration_completed` when coverage ends. The Python node processes LiDAR and camera data, runs YOLO classification, and coordinates Nav2 goals.

The `run_both_nodes.launch.py` launch file starts the project's C++ and Python nodes. Gazebo, SLAM Toolbox, and Nav2 must be started separately. SLAM Toolbox can build the occupancy grid while the C++ controller follows its coverage pattern.

LiDAR proximity is used to decide when to process a camera frame. YOLO filters detections by confidence and the configured harmful-object classes. The current implementation associates each accepted detection with the robot's estimated map pose at the time of processing; it does not calculate metric object position from the camera image itself. Nearby detections of the same class are merged.

## Features

* ROS2-based modular architecture
* TurtleBot3 simulation in Gazebo
* C++ PD-based lawn-mower coverage control
* Python LiDAR and camera perception
* YOLO-based object classification and detection filtering
* `/perception_ready` handshake: C++ waits for perception initialization before exploring
* `/exploration_completed` notification from C++ to Python
* SLAM Toolbox mapping and localisation
* Occupancy grid generation
* Nav2 return-home and detected-object waypoint navigation
* Optional synthetic detection mode for testing without loading YOLO
* Localisation and trajectory validation tools

## System Architecture

Disclaimer: generated using AI

<img width="1536" height="1024" alt="System Architecture" src="https://github.com/user-attachments/assets/0c448a0b-9aa6-484b-8824-485d2dafe424" />

## Workflow

1. Start Gazebo, SLAM Toolbox, and Nav2 with compatible simulation-time and TF settings.
2. Start both project nodes with `ros2 launch nav_with_me run_both_nodes.launch.py`.
3. Python loads YOLO and publishes `true` on `/perception_ready`; in fake mode it skips model loading and publishes the same readiness signal. C++ holds the robot stopped until this signal arrives.
4. The C++ controller uses LiDAR and odometry to execute the lawn-mower path and publishes `true` on `/exploration_completed` when finished.
5. Python sends a Nav2 goal to the starting position, then visits waypoints generated from detected-object poses, and finally sends one return-home goal.

To test the detection and waypoint flow without loading YOLO, launch with:

```bash
ros2 launch nav_with_me run_both_nodes.launch.py use_fake_detections:=true
```

Fake mode generates a synthetic `knife` detection near the robot's current estimated pose. It tests the software flow, not camera detection accuracy or true object localization.

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
│   │   ├── nav2_goal_manager.py
│   │   ├── object_detection.py
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

Lawn-mower coverage pattern with navigation home:

https://github.com/user-attachments/assets/527cadc1-018b-45ed-b41b-63e910991c1a


Detected objects SLAM Map:

<img width="425" height="434" alt="detected objects vs map" src="https://github.com/user-attachments/assets/26987a86-0a6c-42bc-9b95-db55fa10dac4" />

Waypoint Accuracy Validation:

<img width="352" height="278" alt="trajectory vs estimated" src="https://github.com/user-attachments/assets/8b4d6c8c-3f05-44ac-a201-7fc22e5f966a" />

Front wall detection and stopping:

https://github.com/user-attachments/assets/47f3c07c-85de-4b2b-b749-4bcda1794aff

## Author

Joud Salhi

Robotics, Autonomous Systems, ROS2 and Software Engineering

