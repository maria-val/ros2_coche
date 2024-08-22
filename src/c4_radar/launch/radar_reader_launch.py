from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="c4_radar",
            executable="radar_reader",
            name="ars408",
            output="screen",
            emulate_tty=True,
            parameters=[
              {"ip": "192.168.254.254"},
              {"port": 2001}
            ]
        ),
        Node(
            package="c4_radar",
            executable="radar_reader",
            name="srr208",
            output="screen",
            emulate_tty=True,
            parameters=[
              {"ip": "192.168.254.250"},
              {"port": 2001}
            ]
        )
    ])