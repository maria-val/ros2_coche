from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="c4_radar",
            executable="radar_reader",
            output="screen",
            emulate_tty=True,
            parameters=[
              {"ip": "192.168.254.250"},
              {"port": 2001}
            ]
        )
    ])