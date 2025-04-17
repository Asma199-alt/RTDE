# RTDE - Universal Robot Control

## Overview
This repository contains libraries and scripts for controlling Universal Robots using the Real-Time Data Exchange (RTDE) interface. The project allows users to save robot positions to a JSON file and execute predefined pick and place operations through simple, reusable movement functions.

## Features
- Position capture and storage in JSON format
- Pick and place operations using saved positions
- Joint and linear movement control
- TCP position monitoring and adjustment
- Basic gripper control functionality (in development)
- Utility functions for rotation and position transformations

## Files and Components

### Main Scripts
- **test_main.py**: Main control script that performs pick and place operations using movej commands. Moves the robot through 5 predefined positions.
- **pos_capture.py**: Tool for capturing and saving robot positions to the robot_positions.json file.
- **test_gripper.py**: Testing script for the robot's gripper functionality.
- **util.py**: Utility functions for rotation vector conversions and other mathematical operations.

### Data Files
- **robot_positions.json**: Stores captured robot positions including TCP poses and joint positions.

## Usage

### Capturing Robot Positions
```python
# Run the position capture tool
python pos_capture.py

# Follow prompts to:
# 1. Name each position
# 2. Move the robot to desired position
# 3. Capture and save
```

### Running Pick and Place Operations
```python
# Execute the main test script to run through saved positions
python test_main.py
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

## Technical Details
The project communicates with Universal Robots using:
- Socket communication on port 30003
- RTDE interface for real-time data exchange
- TCP/IP commands for robot control

## Requirements
- Python 3.6+
- Universal Robot with RTDE support
- Network connection to the robot

## Setup
1. Ensure network connectivity with the robot
2. Update the robot IP in the scripts (currently set to 129.244.149.108)
3. Run position capture to define your workspace positions
4. Execute test scripts to verify functionality

## Future Development
- Complete gripper open/close functionality
- Add force control and sensing capabilities
- Implement error recovery protocols
- Create a user interface for easier operation

## Author
Baaer and contributors

## License
Proprietary - All rights reserved