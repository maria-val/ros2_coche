from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="c4_radar",
            executable="radar_parser",
            name="radar_408",
            output="screen",
            emulate_tty=True,
            parameters=[
              {"header": "t60"},
              {"topic_publisher": "radar_parser/ars408"}
            ]),
        Node(
            package="c4_radar",
            executable="radar_parser",
            name="radar_208_t61",
            output="screen",
            emulate_tty=True,
            parameters=[
              {"header": "t61"},
              {"topic_publisher": "radar_parser/t61"}
            ]),
        Node(
            package="c4_radar",
            executable="radar_parser",
            name="radar_208_t62",
            output="screen",
            emulate_tty=True,
            parameters=[
              {"header": "t62"},
              {"topic_publisher": "radar_parser/t62"}
            ])
    ])