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

# Install UDEV rules
# UDEV rules are necessary that you can access the serial port without an additonal command
cd ~/ros2_ws/src/resqbot_drive_interface
sudo cp 99-arduino-udev.rules /etc/udev/rules.d/

# Add user to group dialout
sudo usermod -aG dialout $USER

# Restart system (only necessary after first installation)
sudo reboot

# Build package
cd ~/ros2_ws
colcon build

# Source workspace 
source ~/ros2_ws/install/setup.bash

# Run node (with default settings)
ros2 launch resqbot_drive_interface default_launch.py

# If node is running publish a drive speed via ros topic pub command
#  Set left speed to 100 rpm
ros2 topic pub /drive_interface/cmd/speed/left std_msgs/msg/Float32 "data: 100.0"

#  Set right speed to 100 rpm
ros2 topic pub /drive_interface/cmd/speed/right std_msgs/msg/Float32 "data: 100.0"
```

# Parameters

The node can be configured with different parameters. You can simply modify them in the launch file located in the launch/ directory.

*Important note: If you change the launch file you have to run colcon build to copy it from the project directory to your workspace!*

|Parameter|Data-Type|Default-Value|Description|
|-|-|-|-
|update_rate_hz     | Double    | 10.0          | Update rate of the node in [Hz] |
|serial_timeout_sec | Double    | 0.1           | Maximum time for arduino board to response before timeout and reconnect is triggered |
|serial_name        | String    | /dev/ttyACM0  | Serial name of the device |
|serial_baudrate    | Int       | 115200        | Baudrate of the arduino board |

# Arduino Example Program

The program is located in the directory 'arduino/':

```C
//
// ARDUINO UNO Example Program
//
// Received the desired speed from the ros node via the serial line and
// stores it into the global variables. 
// A response is returend for connection alive check.

int16_t speed_left = 0;
int16_t speed_right = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // Read string until \n is received
  String rx_msg = Serial.readStringUntil('\n');

  // Export data from string
  sscanf(rx_msg.c_str(), "L%iR%i\n", &speed_left, &speed_right);

  // Respond to ros node
  Serial.print("Set: Speed-Left: ");
  Serial.print(speed_left);
  Serial.print(" [rpm] Speed Right: ");
  Serial.print(speed_right);
  Serial.println(" [rpm]");
}
```

# TODO

- Add timeout detection for ROS messages and set speed automatically to 0 or stop drives
- Add timeout detection in arduino sketch to stop drives if connection is gone