# ResQBot Drive Interface ROS2 <-> Arduino UNO

# Overview

This node establish a communication between a ros2 network and an arduino uno.

# Install

```bash
# Install python serial
sudo apt update
sudo apt install python3-serial

# Copy repository to ros2_ws/src/ directory
mkdir -p ~/ros2_ws/src/
cd ~/ros2_ws/src
git clone git@github.com:ResQBots/resqbot_drive_interface.git

# Build package
cd ~/ros2_ws
colcon build

# Source workspace 
source ~/ros2_ws/install/setup.bsh

# Run node (with default settings)
ros2 run resqbot_drive_interface drive_interface

# If node is running publish a drive speed via ros topic pub command
#  Set left speed to 100 rpm
ros2 topic pub /cmd/speed/left std_msgs/msg/Float32 "data: 100.0"

#  Set right speed to 100 rpm
ros2 topic pub /cmd/speed/right std_msgs/msg/Float32 "data: 100.0"
```
