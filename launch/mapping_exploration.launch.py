"""
mapping_exploration.launch.py
-------------------------------
Launches slam_toolbox (online async) + Wavefront Frontier Exploration together.

Usage:
  Simulation:
    ros2 launch cleaner_robot_fyp mapping_exploration.launch.py use_sim_time:=true

  Real robot:
    ros2 launch cleaner_robot_fyp mapping_exploration.launch.py use_sim_time:=false

Assumes:
  - temp_sim.launch.py (sim) OR launch_actual.launch.py (real) is already running.
  - nav2 (navigate_to_pose action server) is already running.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'cleaner_robot_fyp'

    # ── Launch arguments ──────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true, real time if false',
    )

    exploration_rate_arg = DeclareLaunchArgument(
        'exploration_rate',
        default_value='1.0',
        description='Seconds between frontier re-evaluations',
    )

    goal_timeout_arg = DeclareLaunchArgument(
        'goal_timeout',
        default_value='30.0',
        description='Seconds before a navigation goal is considered stuck',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    exploration_rate = LaunchConfiguration('exploration_rate')
    goal_timeout = LaunchConfiguration('goal_timeout')

    # ── slam_toolbox ──────────────────────────────────────────────────────────
    slam_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'mapper_params_online_async.yaml',
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── Wavefront Frontier Explorer ───────────────────────────────────────────
    # Delay start slightly so slam_toolbox has time to publish the first /map
    wfe_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='cleaner_robot_fyp',
                executable='wavefront_frontier_exploration',
                name='wavefront_frontier_explorer',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'exploration_rate': exploration_rate,
                        'goal_timeout': goal_timeout,
                    }
                ],
            )
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        exploration_rate_arg,
        goal_timeout_arg,
        slam_toolbox,
        wfe_node,
    ])
