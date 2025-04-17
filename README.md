# RTDE - Universal Robot Control

## Overview
This repository contains libraries and scripts for controlling Universal Robots using the Real-Time Data Exchange (RTDE) interface. The project allows users to save robot positions to a JSON file and execute predefined pick and place operations.

## Features
- Position capture and storage in JSON format file
- Pick and place operations using saved positions
- TCP position monitoring and adjustment
- onRobot RG2 gripper control(in development)


## Files and Components

### Main Scripts
- **test_main.py**: Main control script that performs pick and place operations using movej commands. Moves the robot through predefined positions.
- **pos_capture.py**: Tool for capturing and saving robot positions to the robot_positions.json file.
- **gripperTest.py**: Script for testing and controlling the onRobot RG2 gripper, including opening and closing operations.

### Data Files
- **robot_positions.json**: Stores captured robot positions including TCP poses and joint positions.

## Usage

### Capturing Robot Positions
```python
# Run the position capture tool
python3 pos_capture.py

# Follow prompts to:
# 1. Name each position
# 2. Move the robot to desired position
# 3. Capture and save
```

### Running Pick and Place Operations
```python
# Execute the main test script to run through saved positions
python3 test_main.py
```

### RG2 Gripper Control
```python
# Import the gripper module
import onRobot.gripper as gripper

# Connect to the gripper
rg_id = 0
ip = "192.168.56.101"  # IP address of the gripper
rg_gripper = gripper.RG2(ip, rg_id)

# Get current gripper width
rg_width = rg_gripper.get_rg_width()
print("rg_width: ", rg_width)

# Set gripper force
target_force = 40.00

# Open gripper to 100mm with specified force
rg_gripper.rg_grip(100.0, target_force)
```

### Key Functions

#### Position Control
- `move_to_tcp(target_tcp)`: Move the robot to a target TCP position using linear movement
- `move_to_joint(target_tcp)`: Move the robot to a target TCP position using joint movement
- `increase_move(delta_x, delta_y, delta_z, delta_theta)`: Make incremental movements

#### Position Monitoring
- `get_current_tcp()`: Get the current TCP position
- `get_current_pos()`: Get the current position as x, y, theta

#### Pick and Place
- `pick_and_place_with_saved_positions()`: Execute a sequence of movements using saved positions
