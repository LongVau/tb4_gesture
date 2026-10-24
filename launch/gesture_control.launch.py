"""Launch file starting gesture_recognition_node and robot_behavior_node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    camera_source = LaunchConfiguration("camera_source")
    show_window = LaunchConfiguration("show_window")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_source",
                default_value="0",
                description="Webcam index (0 for /dev/video0) or stream URL",
            ),
            DeclareLaunchArgument(
                "show_window",
                default_value="true",
                description="Show OpenCV gesture recognition window",
            ),
            Node(
                package="tb4_gesture",
                executable="gesture_recognition_node",
                name="gesture_recognition_node",
                output="screen",
                parameters=[
                    {
                        "camera_source": camera_source,
                        "show_window": show_window,
                    }
                ],
            ),
            Node(
                package="tb4_gesture",
                executable="robot_behavior_node",
                name="robot_behavior_node",
                output="screen",
            ),
        ]
    )

