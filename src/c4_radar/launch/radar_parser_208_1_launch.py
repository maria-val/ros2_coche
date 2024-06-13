from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="c4_radar",
            executable="radar_parser",
            output="screen",
            emulate_tty=True,
            parameters=[
              {"header": "t61"},
              {"topic_publisher": "radar_parser/t61"}
            ]
        )
    ])