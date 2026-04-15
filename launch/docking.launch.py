#!/usr/bin/env python3
"""
docking.launch.py
-----------------
Launches the AprilTag-based docking pipeline for cleaner_robot_fyp.
Compatible with both Gazebo simulation and the real robot.

Nodes started:
  1. image_proc::RectifyNode      - rectifies raw camera images (required by apriltag_ros)
  2. AprilTagNode                 - detects tag36h11 tags and publishes TF transforms
  3. detected_dock_pose_publisher - converts tag TF to /detected_dock_pose (PoseStamped)

Camera topics (same topic names for both sim and real robot):
  Simulation : ros_gz_image bridge  -> /camera/image_raw, /camera/camera_info
  Real robot : camera_ros node      -> /camera/image_raw, /camera/camera_info
  (rectified output)                -> /camera/image_rect

TF frames used:
  - parent_frame: camera_link_optical   (camera.xacro camera_optical_joint)
  - child_frame:  tag36h11:0            (published by apriltag_ros for tag ID 0)

Usage:
  # Real robot (default):
  ros2 launch cleaner_robot_fyp docking.launch.py

  # Gazebo simulation:
  ros2 launch cleaner_robot_fyp docking.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory('cleaner_robot_fyp')

    # ── Launch arguments ─────────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use Gazebo simulation clock (true) or wall clock (false). '
                     'Pass use_sim_time:=true when running in Gazebo simulation.'
    )
    declare_tag_family = DeclareLaunchArgument(
        name='tag_family',
        default_value='tag36h11',
        description='AprilTag family'
    )
    declare_tag_id = DeclareLaunchArgument(
        name='tag_id',
        default_value='0',
        description='ID of the AprilTag mounted on the docking station'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    tag_family   = LaunchConfiguration('tag_family')
    tag_id       = LaunchConfiguration('tag_id')

    apriltag_config = os.path.join(pkg_share, 'config', 'apriltags_36h11.yaml')

    # ── 1. image_proc rectify node ────────────────────────────────────────────
    # apriltag_ros expects a rectified image on <ns>/image_rect.
    # Our Gazebo bridge publishes raw images on /camera/image_raw and
    # camera_info on /camera/camera_info.
    rectify_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='rectify_node',
        namespace='camera',
        remappings=[
            ('image',       'image_raw'),    # input:  /camera/image_raw
            ('image_rect',  'image_rect'),   # output: /camera/image_rect
            ('camera_info', 'camera_info'),  # input:  /camera/camera_info
        ],
        parameters=[{
            'queue_size':        5,
            'interpolation':     1,
            'use_sim_time':      use_sim_time,
            'image_transport':   'raw',
        }],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # ── 2. apriltag_ros detector node ─────────────────────────────────────────
    # Publishes TF: camera_link_optical → tag36h11:0
    apriltag_node = ComposableNode(
        package='apriltag_ros',
        plugin='AprilTagNode',
        name='apriltag_node',
        namespace='camera',
        remappings=[
            ('image_rect',   'image_rect'),   # from rectify above
            ('camera_info',  'camera_info'),
        ],
        parameters=[
            apriltag_config,
            {'use_sim_time': use_sim_time},
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # ── Component container (hosts rectify + apriltag) ────────────────────────
    apriltag_container = ComposableNodeContainer(
        name='apriltag_docking_container',
        namespace='camera',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            rectify_node,
            apriltag_node,
        ],
        output='screen'
    )

    # ── 3. detected_dock_pose_publisher ───────────────────────────────────────
    # Reads TF tree and publishes /detected_dock_pose (PoseStamped)
    detected_dock_pose_publisher = Node(
        package='cleaner_robot_fyp',
        executable='detected_dock_pose_publisher',
        name='detected_dock_pose_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            # Must match camera.xacro camera_optical_joint child link
            'parent_frame': 'camera_link_optical',
            # apriltag_ros names the frame: <family>:<id>
            'child_frame':  [tag_family, ':', tag_id],
            'publish_rate': 10.0,
        }],
        output='screen'
    )

    # ── Assemble launch description ───────────────────────────────────────────
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_tag_family)
    ld.add_action(declare_tag_id)
    ld.add_action(apriltag_container)
    ld.add_action(detected_dock_pose_publisher)
    return ld
