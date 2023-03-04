from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='resqbot_drive_interface',
            namespace='drive_interface',
            executable='drive_interface',
            name='drive_interface',
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "update_rate_hz": 0.2,
                    "serial_timeout_sec": 0.1,
                    "serial_name": "/dev/ttyACM0",
                    "serial_baudrate": 115200,
                }
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ])