# import os

# from ament_index_python.packages import get_package_share_directory


# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration

# from launch_ros.actions import Node



# def generate_launch_description():


#     # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
#     # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

#     package_name='cleaner_robot_fyp' #<--- CHANGE ME

#     use_ros2_control = LaunchConfiguration('use_ros2_control')

#     rsp = IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource([os.path.join(
#                     get_package_share_directory(package_name),'launch','rsp.launch.py'
#                 )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': use_ros2_control}.items()
#     )
    
#     world = LaunchConfiguration('world')

#     world_arg = DeclareLaunchArgument(
#         'world',
#         default_value="empty.sdf",
#         description='World to load'
#         )

#     # Include the Gazebo launch file, provided by the ros_gz_sim package
#     gazebo = IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource([os.path.join(
#                     get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
#                     launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
#              )

#     # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
#     spawn_entity = Node(package='ros_gz_sim', executable='create',
#                         arguments=['-topic', 'robot_description',
#                                    '-name', 'my_bot',
#                                    '-z', '0.0335'],
#                         output='screen')


#     # Launch the ROS-Gazebo bridge for normal topics
#     bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
#     ros_gz_bridge = Node(
#         package="ros_gz_bridge",
#         executable="parameter_bridge",
#         arguments=[
#             '--ros-args',
#             '-p',
#             f'config_file:={bridge_params}',
#         ]
#     )

#     ros_gz_image_bridge = Node(
#         package="ros_gz_image",
#         executable="image_bridge",
#         arguments=["/camera/image_raw"]
#     )

#     diff_drive_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=[
#             "diff_cont",
#             '--controller-ros-args',
#             '-r /diff_cont/cmd_vel:=/cmd_vel'
#         ],
#     )

#     joint_broad_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_broad"],
#     )

#     twist_mux_config = os.path.join(get_package_share_directory(package_name),
#                                          'config', 'twist_mux.yaml')
#     twist_mux = Node(
#         package='twist_mux',
#         executable='twist_mux',
#         output='screen',
#         remappings={('/cmd_vel_out', '/cmd_vel')},
#         parameters=[
#             {'use_sim_time': True},
#             twist_mux_config])

#     # Launch them all!
#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'use_ros2_control',
#             default_value='true',
#             description='Use ros2_control if true'),
#         rsp,
#         world_arg,
#         gazebo,
#         spawn_entity,
#         ros_gz_bridge,
#         ros_gz_image_bridge,
#         diff_drive_spawner,
#         joint_broad_spawner,
#         twist_mux,

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
                             RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """
    temp_sim.launch.py

      mode:=sim_only   (default) — Gazebo + robot only. Your original 6-step
                                   known-map workflow is completely unchanged.

      mode:=mapping              — Gazebo + robot + slam_toolbox + nav2 + WFE.
                                   Fully automated mapping, one command.
                                   Startup sequence is event-driven:
                                     1. Gazebo + robot (immediate)
                                     2. slam_toolbox   (after 5 s, robot settled)
                                     3. nav2           (after /map published)
                                     4. WFE            (after nav2 is up, +20 s)
                                     5. map save       (when WFE exits after completion)

      mode:=navigation           — Gazebo + robot + amcl + nav2.
                                   Replaces your manual steps 3 & 5.
                                   Requires map:= argument.
                                   You still do steps 2, 4, 6 in RViz.

      mode:=coverage             — Gazebo + robot + amcl + nav2 + IPA coverage.
                                   Requires map:= argument.
                                   Can autostart a room-coverage request and
                                   execute it via Nav2 waypoint following.

    Examples:
      ros2 launch cleaner_robot_fyp temp_sim.launch.py world:=demo_room.sdf
      ros2 launch cleaner_robot_fyp temp_sim.launch.py mode:=mapping \\
          world:=demo_room.sdf save_map:=/home/cj-ubuntu/dev_ws/demo_room_map \\
          spawn_x:=-3.45 spawn_y:=-2.5 spawn_yaw:=3.14
      ros2 launch cleaner_robot_fyp temp_sim.launch.py mode:=navigation \\
          map:=/home/cj-ubuntu/dev_ws/room_map_save.yaml world:=demo_room.sdf
      ros2 launch cleaner_robot_fyp temp_sim.launch.py mode:=coverage \\
          map:=/home/cj-ubuntu/dev_ws/room_map_save.yaml world:=demo_room.sdf \\
          coverage_autostart:=true coverage_auto_dock:=true
    """

    package_name = 'cleaner_robot_fyp'
    pkg_share    = get_package_share_directory(package_name)

    launch_start_wall_time = time.time()

    use_ros2_control = LaunchConfiguration('use_ros2_control')
    world            = LaunchConfiguration('world')
    mode             = LaunchConfiguration('mode')
    map_yaml         = LaunchConfiguration('map')
    save_map         = LaunchConfiguration('save_map')
    coverage_autostart = LaunchConfiguration('coverage_autostart')
    coverage_auto_dock = LaunchConfiguration('coverage_auto_dock')
    coverage_dock_id = LaunchConfiguration('coverage_dock_id')
    coverage_behavior_tree = LaunchConfiguration('coverage_behavior_tree')
    coverage_undock_on_start = LaunchConfiguration('coverage_undock_on_start')
    coverage_undock_dock_type = LaunchConfiguration('coverage_undock_dock_type')
    spawn_x          = LaunchConfiguration('spawn_x')
    spawn_y          = LaunchConfiguration('spawn_y')
    spawn_z          = LaunchConfiguration('spawn_z')
    spawn_yaw        = LaunchConfiguration('spawn_yaw')

    # ── Always: robot base ────────────────────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': use_ros2_control}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_bot',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
        output='screen',
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['--ros-args', '-p',
                   f'config_file:={os.path.join(pkg_share, "config", "gz_bridge.yaml")}'],
    )

    ros_gz_image_bridge = Node(
        package='ros_gz_image', executable='image_bridge',
        arguments=['/camera/image_raw'],
    )

    diff_drive_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['diff_cont', '--controller-ros-args', '-r /diff_cont/cmd_vel:=/cmd_vel -r /diff_cont/odom:=/odom'], # added /diff_cont/odom:=/odom
    )

    joint_broad_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_broad'],
    )

    twist_mux = Node(
        package='twist_mux', executable='twist_mux', output='screen',
        remappings={('/cmd_vel_out', '/cmd_vel')},
        parameters=[{'use_sim_time': True},
                    os.path.join(pkg_share, 'config', 'twist_mux.yaml')],
    )

    # ═════════════════════════════════════════════════════════════════════════
    # MODE: mapping
    # Sequence: robot → slam_toolbox (t+5s) → [/map received] → nav2 → [+20s] → WFE
    # ═════════════════════════════════════════════════════════════════════════

    # Step 1: slam_toolbox, delayed 5 s so Gazebo/odom TF is publishing
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml'),
            {'use_sim_time': True},
        ],
    )

    # Step 2: map_ready_guard — exits as soon as slam_toolbox publishes /map
    map_guard = Node(
        package='cleaner_robot_fyp',
        executable='map_ready_guard',
        name='map_ready_guard',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Step 3: nav2 — triggered by map_ready_guard exiting (map frame guaranteed)
    nav2_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch', 'navigation_launch.py')]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        }.items()
    )

    # Step 4: WFE — 20 s after nav2 starts (nav2 lifecycle takes ~10-15 s)
    wfe_node = Node(
        package='cleaner_robot_fyp',
        executable='wavefront_frontier_exploration',
        name='wavefront_frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'exploration_rate': 1.0,
        }],
    )

    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='auto_map_saver',
        output='screen',
        arguments=['-f', save_map],
        parameters=[{
            'use_sim_time': True,
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
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['slam_toolbox'],
        }],
    )

    mapping_group = GroupAction(
        condition=LaunchConfigurationEquals('mode', 'mapping'),
        actions=[
            # slam_toolbox + its lifecycle manager start after 5 s
            TimerAction(period=5.0, actions=[
                LogInfo(msg='[mapping] 5-second timer fired — launching slam_toolbox and map_guard'),
                slam_toolbox_node,
                slam_lifecycle_manager,
                map_guard,
            ]),
            # nav2 fires the moment map_guard exits (map TF exists)
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=map_guard,
                    on_exit=[nav2_action],
                )
            ),
            # WFE fires 20 s after map_guard exits (nav2 lifecycle complete)
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
            )
        ]
    )

    # ═════════════════════════════════════════════════════════════════════════
    # MODE: navigation (known map)
    # Sequence: robot → amcl (t+5s) → [/map received] → nav2
    # ═════════════════════════════════════════════════════════════════════════

    amcl_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch', 'localization_launch.py')]),
        launch_arguments={'map': map_yaml, 'use_sim_time': 'true'}.items()
    )

    # Same guard reused — waits for amcl to publish /map
    map_guard_nav = Node(
        package='cleaner_robot_fyp',
        executable='map_ready_guard',
        name='map_ready_guard',
        output='screen',
    )

    nav_ready_guard_nav = Node(
        package='cleaner_robot_fyp',
        executable='nav_ready_guard',
        name='nav_ready_guard_navigation',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    nav2_action_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch', 'navigation_launch.py')]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        }.items()
    )

    navigation_group = GroupAction(
        condition=LaunchConfigurationEquals('mode', 'navigation'),
        actions=[
            TimerAction(period=5.0, actions=[amcl_action, map_guard_nav]),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=map_guard_nav,
                    on_exit=[nav_ready_guard_nav],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=nav_ready_guard_nav,
                    on_exit=[nav2_action_nav],
                )
            ),
        ]
    )

    # ═════════════════════════════════════════════════════════════════════════
    # MODE: coverage (known map + localization + nav2 + IPA room exploration)
    # Sequence: robot → amcl (t+5s) → [/map received] → nav2 + IPA servers
    #                           → optional coverage request after bringup
    # ═════════════════════════════════════════════════════════════════════════

    map_guard_coverage = Node(
        package='cleaner_robot_fyp',
        executable='map_ready_guard',
        name='map_ready_guard_coverage',
        output='screen',
    )

    nav_ready_guard_coverage = Node(
        package='cleaner_robot_fyp',
        executable='nav_ready_guard',
        name='nav_ready_guard_coverage',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    nav2_action_coverage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch', 'navigation_launch.py')]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        }.items()
    )

    docking_perception_coverage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'docking.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items(),
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
            '-p', 'use_sim_time:=true',
        ],
        output='screen',
    )

    coverage_manager = Node(
        package='cleaner_robot_fyp',
        executable='coverage_manager',
        name='coverage_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
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
            'use_sim_time': True,
            'coverage_radius': 0.20,
            'reachable_clearance_radius': 0.20,
            'publish_topic': '/coverage/traversed_map',
            'robot_track_topic': '/coverage/robot_track',
        }],
    )

    coverage_group = GroupAction(
        condition=LaunchConfigurationEquals('mode', 'coverage'),
        actions=[
            TimerAction(period=5.0, actions=[amcl_action, map_guard_coverage]),
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
                        nav2_action_coverage,
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
        DeclareLaunchArgument('use_ros2_control', default_value='true'),
        DeclareLaunchArgument('world', default_value='empty.sdf',
                              description='World SDF file to load'),
        DeclareLaunchArgument('mode', default_value='sim_only',
                              description='sim_only | mapping | navigation | coverage'),
        DeclareLaunchArgument('map', default_value='',
                              description='Path to map YAML (required for mode:=navigation or mode:=coverage)'),
        DeclareLaunchArgument('save_map',
                              default_value='/home/cj-ubuntu/dev_ws/auto_exploration_map',
                              description='Output map path prefix for mode:=mapping'),
        DeclareLaunchArgument(
            'spawn_x',
            default_value='0.0',
            description='Initial robot spawn x position in the Gazebo world frame',
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='0.0',
            description='Initial robot spawn y position in the Gazebo world frame',
        ),
        DeclareLaunchArgument(
            'spawn_z',
            default_value='0.0335',
            description='Initial robot spawn z position in the Gazebo world frame',
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='0.0',
            description='Initial robot spawn yaw in the Gazebo world frame, radians',
        ),
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
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        ros_gz_image_bridge,
        diff_drive_spawner,
        joint_broad_spawner,
        twist_mux,
        mapping_group,
        navigation_group,
        coverage_group,
    ])
