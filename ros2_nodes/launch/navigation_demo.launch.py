#!/usr/bin/env python3

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def rust_node_process(
    pkg_dir: str,
    node_name: str,
    *,
    condition=None,
    additional_env=None,
) -> ExecuteProcess:
    binary_path = os.path.join(pkg_dir, node_name, "target", "release", node_name)
    return ExecuteProcess(
        cmd=[binary_path],
        name=node_name,
        output="screen",
        condition=condition,
        additional_env=additional_env,
    )


def python_script_process(
    script_path: str,
    name: str,
    *,
    args=None,
    condition=None,
) -> ExecuteProcess:
    cmd = ["python3", script_path]
    if args:
        cmd.extend(args)
    return ExecuteProcess(
        cmd=cmd,
        name=name,
        output="screen",
        condition=condition,
    )


def generate_launch_description() -> LaunchDescription:
    ros2_nodes_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_rviz_config = os.path.join(ros2_nodes_dir, "launch", "navigation_demo.rviz")
    odom_tf_broadcaster = os.path.join(ros2_nodes_dir, "launch", "odom_tf_broadcaster.py")
    map_odom_tf_broadcaster = os.path.join(
        ros2_nodes_dir, "launch", "map_odom_tf_broadcaster.py"
    )
    biased_odom_publisher = os.path.join(
        ros2_nodes_dir, "launch", "biased_odom_publisher.py"
    )
    slam_ground_truth_monitor = os.path.join(
        ros2_nodes_dir, "launch", "slam_ground_truth_monitor.py"
    )
    turtlebot3_launch_dir = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "launch",
    )
    ros_gz_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py",
    )
    turtlebot3_world = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "worlds",
        "turtlebot3_world.world",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("turtlebot3_model", default_value="burger"),
            DeclareLaunchArgument("spawn_x", default_value="-2.0"),
            DeclareLaunchArgument("spawn_y", default_value="-0.5"),
            DeclareLaunchArgument("enable_gazebo_gui", default_value="true"),
            DeclareLaunchArgument("publish_map_odom_tf", default_value="false"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("enable_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("enable_ekf_localizer", default_value="false"),
            DeclareLaunchArgument("raw_odom_topic", default_value="/odom"),
            DeclareLaunchArgument("slam_input_odom_topic", default_value="/odom"),
            DeclareLaunchArgument("enable_slam_input_odom_bias", default_value="false"),
            DeclareLaunchArgument("slam_input_odom_xy_scale", default_value="1.0"),
            DeclareLaunchArgument("slam_input_odom_yaw_scale", default_value="1.0"),
            DeclareLaunchArgument("slam_input_odom_yaw_bias_rad", default_value="0.0"),
            DeclareLaunchArgument(
                "slam_input_odom_yaw_bias_per_meter", default_value="0.0"
            ),
            DeclareLaunchArgument("base_tf_odom_topic", default_value="/odom"),
            DeclareLaunchArgument("enable_nav_tf_broadcaster", default_value="true"),
            DeclareLaunchArgument("nav_odom_topic", default_value="/odom"),
            DeclareLaunchArgument("nav_global_frame", default_value="odom"),
            DeclareLaunchArgument("enable_slam_corrected_frame", default_value="false"),
            DeclareLaunchArgument("enable_slam_map_odom_tf", default_value="false"),
            DeclareLaunchArgument("slam_pose_topic", default_value="/slam_pose"),
            DeclareLaunchArgument("slam_odom_topic", default_value="/slam_odom"),
            DeclareLaunchArgument("slam_diagnostics_topic", default_value="/slam_diagnostics"),
            DeclareLaunchArgument("enable_slam_ground_truth_monitor", default_value="false"),
            DeclareLaunchArgument(
                "ground_truth_gz_pose_topic",
                default_value="/world/default/dynamic_pose/info",
            ),
            DeclareLaunchArgument("ground_truth_entity_name", default_value="burger"),
            DeclareLaunchArgument(
                "slam_ground_truth_status_topic",
                default_value="/slam_ground_truth_status",
            ),
            DeclareLaunchArgument("dwa_goal_threshold", default_value="0.3"),
            DeclareLaunchArgument("enable_waypoint_navigator", default_value="false"),
            DeclareLaunchArgument(
                "waypoint_mission", default_value="0.4,0.0;0.1,0.4"
            ),
            DeclareLaunchArgument("waypoint_frame", default_value="map"),
            DeclareLaunchArgument("waypoint_loop", default_value="false"),
            DeclareLaunchArgument("waypoint_goal_tolerance", default_value="0.35"),
            SetEnvironmentVariable(
                "TURTLEBOT3_MODEL", LaunchConfiguration("turtlebot3_model")
            ),
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                os.path.join(get_package_share_directory("turtlebot3_gazebo"), "models"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ros_gz_launch),
                launch_arguments={
                    "gz_args": f"-r -s -v2 {turtlebot3_world}",
                    "on_exit_shutdown": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ros_gz_launch),
                launch_arguments={
                    "gz_args": "-g -v2",
                    "on_exit_shutdown": "true",
                }.items(),
                condition=IfCondition(LaunchConfiguration("enable_gazebo_gui")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(turtlebot3_launch_dir, "robot_state_publisher.launch.py")
                ),
                launch_arguments={"use_sim_time": "true"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(turtlebot3_launch_dir, "spawn_turtlebot3.launch.py")
                ),
                launch_arguments={
                    "x_pose": LaunchConfiguration("spawn_x"),
                    "y_pose": LaunchConfiguration("spawn_y"),
                }.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="cmd_vel_twist_bridge",
                arguments=["/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_bridge",
                arguments=[
                    "--frame-id",
                    LaunchConfiguration("map_frame"),
                    "--child-frame-id",
                    LaunchConfiguration("odom_frame"),
                ],
                condition=IfCondition(LaunchConfiguration("publish_map_odom_tf")),
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="navigation_demo_rviz",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("enable_rviz")),
                output="screen",
            ),
            TimerAction(
                period=5.0,
                actions=[
                    python_script_process(
                        biased_odom_publisher,
                        "biased_odom_publisher",
                        args=[
                            "--input-topic",
                            LaunchConfiguration("raw_odom_topic"),
                            "--output-topic",
                            LaunchConfiguration("slam_input_odom_topic"),
                            "--xy-scale",
                            LaunchConfiguration("slam_input_odom_xy_scale"),
                            "--yaw-scale",
                            LaunchConfiguration("slam_input_odom_yaw_scale"),
                            "--yaw-bias-rad",
                            LaunchConfiguration("slam_input_odom_yaw_bias_rad"),
                            "--yaw-bias-per-meter",
                            LaunchConfiguration("slam_input_odom_yaw_bias_per_meter"),
                        ],
                        condition=IfCondition(
                            LaunchConfiguration("enable_slam_input_odom_bias")
                        ),
                    ),
                    rust_node_process(
                        ros2_nodes_dir,
                        "slam_node",
                        additional_env={
                            "SLAM_INPUT_ODOM_TOPIC": LaunchConfiguration(
                                "slam_input_odom_topic"
                            ),
                            "SLAM_OUTPUT_POSE_TOPIC": LaunchConfiguration("slam_pose_topic"),
                            "SLAM_OUTPUT_ODOM_TOPIC": LaunchConfiguration("slam_odom_topic"),
                            "SLAM_DIAGNOSTICS_TOPIC": LaunchConfiguration(
                                "slam_diagnostics_topic"
                            ),
                            "SLAM_USE_CORRECTED_FRAME": LaunchConfiguration(
                                "enable_slam_corrected_frame"
                            ),
                            "SLAM_CORRECTED_FRAME_ID": LaunchConfiguration("map_frame"),
                        },
                    ),
                    rust_node_process(
                        ros2_nodes_dir,
                        "ekf_localizer_node",
                        condition=IfCondition(
                            LaunchConfiguration("enable_ekf_localizer")
                        ),
                        additional_env={
                            "EKF_INPUT_ODOM_TOPIC": "/odom",
                            "EKF_OUTPUT_ODOM_TOPIC": LaunchConfiguration("raw_odom_topic"),
                            "EKF_OUTPUT_POSE_TOPIC": "/ekf_pose",
                        },
                    ),
                    rust_node_process(
                        ros2_nodes_dir,
                        "path_planner_node",
                        additional_env={
                            "RUST_NAV_ODOM_TOPIC": LaunchConfiguration("nav_odom_topic")
                        },
                    ),
                    rust_node_process(
                        ros2_nodes_dir,
                        "dwa_planner_node",
                        additional_env={
                            "RUST_NAV_ODOM_TOPIC": LaunchConfiguration("nav_odom_topic"),
                            "DWA_GOAL_THRESHOLD": LaunchConfiguration("dwa_goal_threshold"),
                        },
                    ),
                    python_script_process(
                        odom_tf_broadcaster,
                        "odom_tf_broadcaster",
                        args=["--odom-topic", LaunchConfiguration("base_tf_odom_topic")],
                        condition=IfCondition(
                            LaunchConfiguration("enable_nav_tf_broadcaster")
                        ),
                    ),
                    python_script_process(
                        map_odom_tf_broadcaster,
                        "map_odom_tf_broadcaster",
                        args=[
                            "--map-odom-topic",
                            LaunchConfiguration("slam_odom_topic"),
                            "--raw-odom-topic",
                            LaunchConfiguration("raw_odom_topic"),
                        ],
                        condition=IfCondition(
                            LaunchConfiguration("enable_slam_map_odom_tf")
                        ),
                    ),
                    python_script_process(
                        slam_ground_truth_monitor,
                        "slam_ground_truth_monitor",
                        args=[
                            "--raw-odom-topic",
                            LaunchConfiguration("raw_odom_topic"),
                            "--slam-odom-topic",
                            LaunchConfiguration("slam_odom_topic"),
                            "--gz-pose-topic",
                            LaunchConfiguration("ground_truth_gz_pose_topic"),
                            "--ground-truth-entity-name",
                            LaunchConfiguration("ground_truth_entity_name"),
                            "--status-topic",
                            LaunchConfiguration("slam_ground_truth_status_topic"),
                        ],
                        condition=IfCondition(
                            LaunchConfiguration("enable_slam_ground_truth_monitor")
                        ),
                    ),
                    rust_node_process(
                        ros2_nodes_dir,
                        "waypoint_navigator_node",
                        condition=IfCondition(
                            LaunchConfiguration("enable_waypoint_navigator")
                        ),
                        additional_env={
                            "RUST_NAV_ODOM_TOPIC": LaunchConfiguration("nav_odom_topic"),
                            "WAYPOINT_NAV_WAYPOINTS": LaunchConfiguration(
                                "waypoint_mission"
                            ),
                            "RUST_NAV_GLOBAL_FRAME": LaunchConfiguration(
                                "nav_global_frame"
                            ),
                            "WAYPOINT_NAV_FRAME": LaunchConfiguration("waypoint_frame"),
                            "WAYPOINT_NAV_LOOP": LaunchConfiguration("waypoint_loop"),
                            "WAYPOINT_NAV_GOAL_TOLERANCE": LaunchConfiguration(
                                "waypoint_goal_tolerance"
                            ),
                        },
                    ),
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
