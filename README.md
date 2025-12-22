# UW GIX MSTI TECHIN 516 Winter 2025 Example Final Project

![project example](/assets/final_project_example.jpg)

## Description

This year, UW GIX MSTI robotics students will designing and developing their own robots!  
Together in teams of {team size}, they will modify Turtlebot 3 robots for new applications.  
They will add 1 new sensor, and 1 new motor to accomplish a task of their choosing.  
This project aims to provide experience developing novel and complete robot systems spanning hardware, software, design, controls, and user interaction.

This example demonstrates a Turtlebot 3 Burger with an added camera and dog treat dispensing motor.  
When the camera sees a dog, the added motor dispenses a treat.  


## Video Demo

[![video demo](https://img.youtube.com/vi/KoSGbtOjZQA/0.jpg)](https://youtu.be/KoSGbtOjZQA)


## Installation

### OpenCR

This project requires cutom firmware for the OpenCR board to drive the 3rd motor.  
Follow the instructions from [this repo](https://github.com/GIXLabs/t516_OpenCR) to install the new firmware.  


### Turtlebot 3 Raspberry Pi

Clone this repo into the Pis ros2 workspace's `src` directory and build:
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

For the above demo, object recognition was performed on my laptop with [this yolo ros package](https://github.com/mgonzs13/yolo_ros).  
Follow their installation instructions to replicate this demo.  


## Usage

### Raspberry Pi

```bash
ros2 launch yolo2motor yolo2motor.launch.py
```

### Laptop

```bash
ros2 launch yolo_bringup yolo.launch.py
```
