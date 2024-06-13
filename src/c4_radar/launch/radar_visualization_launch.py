from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="c4_radar",
            executable="radar_visualizer",
            name="radar_408",
            output="screen",
            emulate_tty=True,
            parameters=[
              {"header": "t60"},
              {"pub_topic": "radar_visualization/ars408"}
            ]),
        Node(
            package="c4_radar",
            executable="radar_visualizer",
            name="radar_208_t61",
            output="screen",
            emulate_tty=True,
            parameters=[
              {"header": "t61"},
              {"pub_topic": "radar_visualization/t61"}
            ]),
        Node(
            package="c4_radar",
            executable="radar_visualizer",
            name="radar_208_t62",
            output="screen",
            emulate_tty=True,
            parameters=[
              {"header": "t62"},
              {"pub_topic": "radar_visualization/t62"}
            ])
    ])