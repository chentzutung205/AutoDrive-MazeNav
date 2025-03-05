# Maze Navigation

This is the final demo of the Autonomous Vehicle project. BB-8, affectionately known as "Beebee-Ate," is a lovable and intelligent droid character from the __Star Wars__ franchise. Inspired by this iconic character, our robot BB-8 is designed to navigate and interact with its environment, showcasing advanced capabilities in object tracking and autonomous navigation.

## Overview

This ROS2 package enables a TurtleBot3 to autonomously navigate a predefined maze while detecting and following signs using computer vision. The navigation is implemented through a combination of SLAM, localization, and real-time decision-making.

## Dependencies

- ROS2 Humble
- Gazebo
- Navigation2 (Nav2)
- OpenCV
- scikit-learn
- geometry_msgs
- sensor_msgs

## Features

- Image classification for recognizing directional signs (left, right, stop, goal, do-not-enter)
- LIDAR-based obstacle detection
- Sign-driven Navigation, relying on a state machine that follows classified signs and makes real-time decisions based on LIDAR and odometry data

## Nodes
**1. part1_knn**

This node trains a kNN model using pre-collected sign images, applies image preprocessing, and saves the trained model for deployment.

**2. trained_knn**

This node uses a trained kNN model to classify signs in real-time.

**3. frontal_viewer**

This node uses LIDAR data to detect obstacles in front of the robot within a 5-degree field of view.

**4. surrounding**

This node detects walls within a 114-degree field of view using LIDAR.

**5. controller**

This node implements a state machine to follow signs and navigate the maze.

## Usage
1. Clone this package into your ROS2 workspace.
2. Ensure all dependencies are installed.
3. Build the package using `colcon build`.
4. Source your workspace.
5. Run the nodes using:
```
ros2 launch bb8_final launch_final.py
```

## Tips
- Use Gazebo for debugging before testing on the physical robot.
- Train the classifier with additional images for better accuracy.
- Introduce delays between navigation steps to improve recognition.
