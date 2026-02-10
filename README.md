# Swinburne Rover Team
## Docker / MicroROS node launch system

This is a controller publisher/subscriber node launch program with GUI dashboard for Swinburne's Rover Team 2025/2026.

## Features

- Xbox Controller & Joystick support for rover control
- Micro-ROS agent for ESP32 communication
- **Real-time Qt5 GUI Dashboard** displaying:
  - Motor drive values for all 4 wheels
  - Wheel steering/rotation with angle display
  - Arm joint positions (Base, Shoulder, Elbow, Wrist, Roll)
  - Gripper state
  - Live rover visualization with wheel orientation

## Quick Start

To run:
First, compile dockerfile (or download it)
```bash
./build.sh
```
or
```bash
docker pull swinroverteam/srt-ros:latest
```

Then, run the setup shell
```bash
./run.sh
```

Everything else should be taken care of for you!

## System Components

1. **Joy Nodes**: Read Xbox controller and joystick input
2. **Controller Node**: Process Xbox controller for drive control
3. **Joystick Node**: Process joystick for arm control
4. **Micro-ROS Agent**: UDP bridge to ESP32
5. **GUI Node**: Qt5-based real-time telemetry dashboard

## GUI Dashboard

The system includes a professional C++ Qt5 GUI that displays:
- Real-time motor and wheel data
- Arm joint positions and gripper state
- Top-down rover visualization with wheel orientation indicators

See [GUI_README.md](GUI_README.md) for detailed GUI documentation.

## ROS 2 Topics

- `/Pivot_Drive` - Motor drive values
- `/Pivot_Rotate` - Wheel steering values
- `/Arm_Joints` - Arm joint positions
- `/Arm_Gripper` - Gripper state

## Requirements

- Docker
- X11 display (for GUI)
- USB devices for controllers (Xbox controller and joystick)
