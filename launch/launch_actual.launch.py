# import os

# from ament_index_python.packages import get_package_share_directory


# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, RegisterEventHandler
# from launch.event_handlers import OnProcessStart
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration


# from launch_ros.actions import Node



# def generate_launch_description():


#     # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
#     # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

#     package_name='cleaner_robot_fyp' #<--- CHANGE ME

#     rsp = IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource([os.path.join(
#                     get_package_share_directory(package_name),'launch','rsp.launch.py'
#                 )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
#     )
    

#     controller_params_file = os.path.join(get_package_share_directory(package_name),'config','controllers_actual.yaml')

#     controller_manager = Node(
#         package='controller_manager',
#         executable='ros2_control_node',
#         parameters=[controller_params_file],
#     )

#     delayed_controller_manager = TimerAction(period=3.0,actions=[controller_manager])


#     diff_drive_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=[
#             "diff_cont",
#             '--controller-ros-args',
#             '-r /diff_cont/cmd_vel:=/cmd_vel'
#         ],
#     )

#     delayed_diff_drive_spawner = RegisterEventHandler(
#         event_handler=OnProcessStart(
#             target_action=controller_manager,
#             on_start=[diff_drive_spawner],
#         )
#     )

#     joint_broad_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_broad"],
#     )

#     delayed_joint_broad_spawner = RegisterEventHandler(
#         event_handler=OnProcessStart(
#             target_action=controller_manager,
#             on_start=[joint_broad_spawner],
#         )
#     )

#     twist_mux_config = os.path.join(get_package_share_directory(package_name),
#         'config', 'twist_mux.yaml')
#     twist_mux = Node(
#         package='twist_mux',
#         executable='twist_mux',
#         output='screen',
#         remappings={('/cmd_vel_out', '/cmd_vel')},
#         parameters=[
#             {'use_sim_time': False},
#             twist_mux_config])
    
#     # ADDED — YDLidar X3
#     lidar = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([os.path.join(
#             get_package_share_directory('ydlidar_ros2_driver'),
#             'launch', 'ydlidar_x3_view_launch.py'
#         )])
#     )

#     # ADDED — Camera node
#     camera = Node(
#         package='camera_ros',
#         executable='camera_node',
#         output='screen',
#         parameters=[{
#             'width': 854,
#             'height': 480,
#         }]
#     )

#     # Launch them all!
#     return LaunchDescription([
#         rsp,
#         delayed_controller_manager,
#         delayed_diff_drive_spawner,
#         delayed_joint_broad_spawner,
#         twist_mux,
#         lidar,
#         camera,
#     ])




import os
import time

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                             IncludeLaunchDescription, LogInfo,
                             RegisterEventHandler,
                             TimerAction)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """
    launch_actual.launch.py

      mode:=robot_only  (default) — Hardware + sensors only. No change.
      mode:=mapping               — Hardware + slam_toolbox + nav2 + WFE.
                                    Saves the map automatically when WFE finishes.
      mode:=navigation            — Hardware + amcl + nav2. Requires map:=.
      mode:=coverage              — Hardware + amcl + nav2 + IPA coverage.
                                    Requires map:=.

    Examples:
      ros2 launch cleaner_robot_fyp launch_actual.launch.py
      ros2 launch cleaner_robot_fyp launch_actual.launch.py mode:=mapping \
          save_map:=/home/cj-ubuntu/dev_ws/real_robot_map
      ros2 launch cleaner_robot_fyp launch_actual.launch.py mode:=navigation \
          map:=/home/cj-ubuntu/dev_ws/real_robot_map.yaml
      ros2 launch cleaner_robot_fyp launch_actual.launch.py mode:=coverage \
          map:=/home/cj-ubuntu/dev_ws/real_robot_map.yaml \
          coverage_autostart:=true coverage_auto_dock:=true
    """

    package_name = 'cleaner_robot_fyp'
    pkg_share    = get_package_share_directory(package_name)
    launch_start_wall_time = time.time()

    mode                         = LaunchConfiguration('mode')
    map_yaml                     = LaunchConfiguration('map')
    save_map                     = LaunchConfiguration('save_map')
    coverage_autostart           = LaunchConfiguration('coverage_autostart')
    coverage_auto_dock           = LaunchConfiguration('coverage_auto_dock')
    coverage_dock_id             = LaunchConfiguration('coverage_dock_id')
    coverage_behavior_tree       = LaunchConfiguration('coverage_behavior_tree')
    coverage_undock_on_start     = LaunchConfiguration('coverage_undock_on_start')
    coverage_undock_dock_type    = LaunchConfiguration('coverage_undock_dock_type')

    # ── Always: hardware base ─────────────────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    controller_manager = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[os.path.join(pkg_share, 'config', 'controllers_actual.yaml')],
    )
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['diff_cont', '--controller-ros-args', '-r /diff_cont/cmd_vel:=/cmd_vel -r /diff_cont/odom:=/odom'], # added /diff_cont/odom:=/odom
    )
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager, on_start=[diff_drive_spawner])
    )

    joint_broad_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_broad'],
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager, on_start=[joint_broad_spawner])
    )

    twist_mux = Node(
        package='twist_mux', executable='twist_mux', output='screen',
        remappings={('/cmd_vel_out', '/cmd_vel')},
        parameters=[{'use_sim_time': False},
                    os.path.join(pkg_share, 'config', 'twist_mux.yaml')],
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ydlidar_ros2_driver'),
            'launch', 'ydlidar_x3_view_launch.py')])
    )

    camera = Node(
        package='camera_ros', executable='camera_node', output='screen',
        parameters=[{'width': 854, 'height': 480}],
    )

    # ═════════════════════════════════════════════════════════════════════════
    # MODE: mapping
    # Sequence: hardware → slam_toolbox (t+8s) → [/map received] → nav2 → [+20s] → WFE
    # ═════════════════════════════════════════════════════════════════════════

    slam_toolbox_node = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml'),
            {'use_sim_time': False},
        ],
    )

    map_guard = Node(
        package='cleaner_robot_fyp', executable='map_ready_guard',
        name='map_ready_guard', output='screen',
        parameters=[{'use_sim_time': False}],
    )

    nav2_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        }.items()
    )

    wfe_node = Node(
        package='cleaner_robot_fyp', executable='wavefront_frontier_exploration',
        name='wavefront_frontier_explorer', output='screen',
        parameters=[{'use_sim_time': False, 'exploration_rate': 1.0}],
    )

    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='auto_map_saver',
        output='screen',
        arguments=['-f', save_map],
        parameters=[{
            'use_sim_time': False,
            'save_map_timeout': 10.0,
            'free_thresh_default': 0.25,
            'occupied_thresh_default': 0.65,
            'map_subscribe_transient_local': True,
        }],
    )

    # Lifecycle manager: configures and activates slam_toolbox automatically
    slam_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['slam_toolbox'],
        }],
    )

    mapping_group = GroupAction(
        condition=LaunchConfigurationEquals('mode', 'mapping'),
        actions=[
            TimerAction(period=8.0, actions=[slam_toolbox_node, slam_lifecycle_manager, map_guard]),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=map_guard,
                    on_exit=[nav2_mapping],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=map_guard,
                    on_exit=[TimerAction(period=20.0, actions=[wfe_node])],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=wfe_node,
                    on_exit=[
                        LogInfo(msg=['[mapping] Exploration finished — saving map to ', save_map]),
                        map_saver_node,
                    ],
                )
            ),
        ]
    )

    # ═════════════════════════════════════════════════════════════════════════
    # MODE: navigation (known map)
    # Sequence: hardware → amcl (t+8s) → [/map received] → nav2
    # ═════════════════════════════════════════════════════════════════════════

    amcl_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')]),
        launch_arguments={'map': map_yaml, 'use_sim_time': 'false'}.items()
    )

    map_guard_nav = Node(
        package='cleaner_robot_fyp', executable='map_ready_guard',
        name='map_ready_guard', output='screen',
        parameters=[{'use_sim_time': False}],
    )

    nav_ready_guard_nav = Node(
        package='cleaner_robot_fyp', executable='nav_ready_guard',
        name='nav_ready_guard_navigation', output='screen',
        parameters=[{'use_sim_time': False}],
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        }.items()
    )

    navigation_group = GroupAction(
        condition=LaunchConfigurationEquals('mode', 'navigation'),
        actions=[
            TimerAction(period=8.0, actions=[amcl_action, map_guard_nav]),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=map_guard_nav,
                    on_exit=[nav_ready_guard_nav],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=nav_ready_guard_nav,
                    on_exit=[nav2_navigation],
                )
            ),
        ]
    )

    # ═════════════════════════════════════════════════════════════════════════
    # MODE: coverage (known map + localization + nav2 + IPA room exploration)
    # Sequence: hardware → amcl (t+8s) → [/map received] → nav2 + IPA servers
    #                           → optional coverage request after bringup
    # ═════════════════════════════════════════════════════════════════════════

    map_guard_coverage = Node(
        package='cleaner_robot_fyp',
        executable='map_ready_guard',
        name='map_ready_guard_coverage',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    nav_ready_guard_coverage = Node(
        package='cleaner_robot_fyp',
        executable='nav_ready_guard',
        name='nav_ready_guard_coverage',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    nav2_coverage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        }.items()
    )

    docking_perception_coverage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'docking.launch.py')]),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(coverage_auto_dock),
    )

    try:
        ipa_room_exploration_prefix = get_package_prefix('ipa_room_exploration')
    except PackageNotFoundError as exc:
        raise RuntimeError(
            'Coverage mode requires the IPA packages to be present in the current '
            'ROS environment. In the same terminal, run '
            '`source /opt/ros/jazzy/setup.bash && source ~/dev_ws/install/setup.bash`, '
            'then verify `ros2 pkg prefix ipa_room_exploration` succeeds before '
            'launching mode:=coverage.'
        ) from exc

    room_exploration_server_executable = os.path.join(
        ipa_room_exploration_prefix, 'lib', 'ipa_room_exploration', 'room_exploration_server'
    )

    room_exploration_server = ExecuteProcess(
        cmd=[
            room_exploration_server_executable,
            '--ros-args',
            '-r', '__ns:=/room_exploration',
            '-r', '__node:=room_exploration_server',
            '--params-file', os.path.join(pkg_share, 'config', 'ipa_room_exploration_params.yaml'),
            '-p', 'use_sim_time:=false',
        ],
        output='screen',
    )

    coverage_manager = Node(
        package='cleaner_robot_fyp',
        executable='coverage_manager',
        name='coverage_manager',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': coverage_autostart,
            'auto_dock_on_completion': coverage_auto_dock,
            'dock_id': coverage_dock_id,
            'coverage_behavior_tree': coverage_behavior_tree,
            'undock_on_start': coverage_undock_on_start,
            'undock_dock_type': coverage_undock_dock_type,
            'continue_on_undock_failure': True,
            'manual_undock_backup_on_failure': True,
            'manual_undock_backup_distance': 0.40,
            'manual_undock_backup_speed': 0.08,
            'robot_radius': 0.22,
            'coverage_radius': 0.20,
            'planning_mode': 1,
            'min_waypoint_spacing': 0.08,
            'segment_sample_distance': 0.12,
            'corner_angle_threshold': 0.25,
            'lateral_deviation_tolerance': 0.02,
            'goal_clearance_radius': 0.24,
            'goal_backoff_distance': 0.30,
            'goal_adjustment_step': 0.05,
            'execution_chunk_size': 2,
            'execution_chunk_distance': 0.8,
            'min_send_goal_distance': 0.25,
            'close_goal_skip_distance': 0.45,
            'close_goal_forward_angle': 1.05,
            'stall_timeout_sec': 10.0,
            'stall_progress_epsilon': 0.2,
            'max_recoveries_per_chunk': 10,
            'always_start_from_beginning': True,
            'enable_escape_reposition': False,
            'max_single_goal_retries_before_defer': 0,
            'max_supplemental_goals_per_pass': 4,
            'supplemental_progress_epsilon': 0.001,
            'max_supplemental_cycles_without_progress': 4,
            'supplemental_sweep_spacing': 0.16,
            'supplemental_min_sweep_length': 0.30,
            'launch_start_wall_time': launch_start_wall_time,
            'current_goal_topic': '/coverage/current_goal',
            'raw_path_topic': '/coverage/raw_path',
            'filtered_path_topic': '/coverage/filtered_path',
            'markers_topic': '/coverage/markers',
        }],
    )

    coverage_progress_tracker = Node(
        package='cleaner_robot_fyp',
        executable='coverage_progress_tracker',
        name='coverage_progress_tracker',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'coverage_radius': 0.20,
            'reachable_clearance_radius': 0.20,
            'publish_topic': '/coverage/traversed_map',
            'robot_track_topic': '/coverage/robot_track',
        }],
    )

    coverage_group = GroupAction(
        condition=LaunchConfigurationEquals('mode', 'coverage'),
        actions=[
            TimerAction(period=8.0, actions=[amcl_action, map_guard_coverage]),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=map_guard_coverage,
                    on_exit=[nav_ready_guard_coverage],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=nav_ready_guard_coverage,
                    on_exit=[
                        LogInfo(msg='[coverage] Map available — starting Nav2 and IPA coverage stack'),
                        nav2_coverage,
                        docking_perception_coverage,
                        room_exploration_server,
                        coverage_manager,
                        coverage_progress_tracker,
                    ],
                )
            ),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='robot_only',
                              description='robot_only | mapping | navigation | coverage'),
        DeclareLaunchArgument('map', default_value='',
                              description='Path to map YAML (required for mode:=navigation or mode:=coverage)'),
        DeclareLaunchArgument('save_map',
                              default_value='/home/cj-ubuntu/dev_ws/real_robot_map',
                              description='Output map path prefix for mode:=mapping'),
        DeclareLaunchArgument(
            'coverage_autostart',
            default_value='false',
            description='Automatically send the IPA room coverage goal in mode:=coverage',
        ),
        DeclareLaunchArgument(
            'coverage_auto_dock',
            default_value='false',
            description='After coverage completes, automatically call /dock_robot',
        ),
        DeclareLaunchArgument(
            'coverage_dock_id',
            default_value='charging_dock',
            description='Dock ID used when coverage_auto_dock:=true',
        ),
        DeclareLaunchArgument(
            'coverage_behavior_tree',
            default_value=os.path.join(pkg_share, 'config', 'coverage_navigate_through_poses.xml'),
            description='Behavior tree used by coverage NavigateThroughPoses goals',
        ),
        DeclareLaunchArgument(
            'coverage_undock_on_start',
            default_value='true',
            description='Undock before coverage autostart planning begins',
        ),
        DeclareLaunchArgument(
            'coverage_undock_dock_type',
            default_value='simple_charging_dock',
            description='Dock plugin type passed to /undock_robot before coverage',
        ),
        rsp,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        twist_mux,
        lidar,
        camera,
        mapping_group,
        navigation_group,
        coverage_group,
    ])
