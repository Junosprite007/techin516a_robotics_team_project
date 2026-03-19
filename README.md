# TECHIN 516 Winter 2025 Final Project

![project photo](/assets/IMG_5790.png)

## Description

Struck Out (ストラックアウト) is a pitching game that originated on a [Japanese TV show](https://ja.wikipedia.org/wiki/%E7%AD%8B%E8%82%89%E7%95%AA%E4%BB%98%E3%82%B7%E3%83%AA%E3%83%BC%E3%82%BA). In this project, we have reimagined the game by implementing an autonomous player using a TurtleBot3.

The system utilizes a camera and the yolo_ros package for real-time object detection. The robot aims at a custom-built target board featuring specific object classes from the COCO dataset.

During gameplay, the robot autonomously selects a target, calculates the required approach distance and firing angle, and attempts to shoot a ping pong ball to knock down the target. To ensure a safe experience, the terminal prompts the user and requires explicit permission before the robot moves or fires. We built this project so people can play and enjoy a fun game alongside a robot.

## Video Demo

[![video demo](https://img.youtube.com/vi/Jw6R5hkiqds/0.jpg)](https://youtu.be/Jw6R5hkiqds)

## Installation

### OpenCR

This project requires cutom firmware for the OpenCR board to drive the 3rd motor.  
Follow the instructions from [this repo](https://github.com/GIXLabs/t516_OpenCR) to install the new firmware.  


### Turtlebot 3 Raspberry Pi

Clone this repo into the Pi's workspace's `src` directory and build:
```bash
cd ~/ros2_ws/src
git clone https://github.com/GIXLabs/t516_project_example.git
cd ..
colcon build
```

After attaching the new motor, flashing the OpenCR, and building the Turtlebot's new workspace, you can test the new motor with the command below:  
Note that positions are in radians.  
**Keep clear of the motor when it moves!**  
```bash
ros2 topic pub --once /gix_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['gix'], points: [{positions: [1.0], time_from_start: {sec: 2}}]}"
```


### Laptop

Object recognition was performed on separate computer with [this yolo ros package](https://github.com/mgonzs13/yolo_ros).  
Follow the installation instructions.  


## Usage

### Raspberry Pi

```bash
ros2 launch yolo2motor yolo2motor.launch.py
ros2 run yolo2motor ping_pong 
```

### Laptop

```bash
ros2 launch yolo_bringup yolo.launch.py
```
