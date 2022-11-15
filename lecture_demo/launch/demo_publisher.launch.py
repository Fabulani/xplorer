#  ------------------------------------------------------------------
#   Copyright (C) Karelics Oy - All Rights Reserved
#   Unauthorized copying of this file, via any medium is strictly
#   prohibited. All information contained herein is, and remains
#   the property of Karelics Oy.
#  ------------------------------------------------------------------

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(_context, *_args, **_kwargs):
    """Setup demo_publisher node to be launched"""

    emulate_tty = LaunchConfiguration("emulate_tty")
    dummy_arg_1 = LaunchConfiguration("dummy_arg_1")
    dummy_arg_2 = LaunchConfiguration("dummy_arg_2")

    demo_publisher_node = Node(
        package="lecture_demo",
        executable="demo_publisher.py",
        name="demo_publisher_node",
        emulate_tty=emulate_tty,
        output="screen",
        parameters=[
            {
                "dummy_arg_1": dummy_arg_1,
                "dummy_arg_2": dummy_arg_2,
            }
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    launch_list = [
        demo_publisher_node,
    ]

    return launch_list


def generate_launch_description():
    """Generate multi-floor launch description"""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "emulate_tty",
                default_value="True",
                description="This argument set to True enables properly the running node to send output to console",
            ),
            DeclareLaunchArgument(
                "dummy_arg_1",
                default_value="1.0",
                description="A dummy launch argument",
            ),
            DeclareLaunchArgument(
                "dummy_arg_2",
                default_value="random_string",
                description="Another dummy launch argument",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
