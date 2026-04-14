#!/usr/bin/env python3

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def rust_node_process(pkg_dir: str, node_name: str) -> ExecuteProcess:
    binary_path = os.path.join(pkg_dir, node_name, "target", "release", node_name)
    return ExecuteProcess(cmd=[binary_path], name=node_name, output="screen")


def generate_launch_description() -> LaunchDescription:
    ros2_nodes_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    turtlebot3_launch = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "launch",
        "turtlebot3_world.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("turtlebot3_model", default_value="burger"),
            SetEnvironmentVariable(
                "TURTLEBOT3_MODEL", LaunchConfiguration("turtlebot3_model")
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(turtlebot3_launch),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="cmd_vel_twist_bridge",
                arguments=["/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"],
                output="screen",
            ),
            TimerAction(
                period=5.0,
                actions=[
                    rust_node_process(ros2_nodes_dir, "slam_node"),
                    rust_node_process(ros2_nodes_dir, "path_planner_node"),
                    rust_node_process(ros2_nodes_dir, "dwa_planner_node"),
                ],
            ),
        ]
    )


def main(argv: list[str] | None = None) -> int:
    launch_service = LaunchService(argv=argv)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
