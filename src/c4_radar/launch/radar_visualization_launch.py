from launch import LaunchDescription
from launch_ros.actions import Node
import os

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
              {"pub_topic": "radar_visualization/t60"}
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
            ]),
        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments = ['--frame-id', 'world', '--child-frame-id', 'map']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(os.environ['C4_RADAR_SRC_PATH'], 'rviz', 'radar_markers_config.rviz')]
        )
    ])