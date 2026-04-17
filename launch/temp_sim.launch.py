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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                             IncludeLaunchDescription, LogInfo,
                             RegisterEventHandler, TimerAction)
from launch.conditions import LaunchConfigurationEquals
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

    Examples:
      ros2 launch cleaner_robot_fyp temp_sim.launch.py world:=demo_room.sdf
      ros2 launch cleaner_robot_fyp temp_sim.launch.py mode:=mapping \\
          world:=demo_room.sdf save_map:=/home/cj-ubuntu/dev_ws/demo_room_map
      ros2 launch cleaner_robot_fyp temp_sim.launch.py mode:=navigation \\
          map:=/home/cj-ubuntu/dev_ws/room_map_save.yaml world:=demo_room.sdf
    """

    package_name = 'cleaner_robot_fyp'
    pkg_share    = get_package_share_directory(package_name)

    use_ros2_control = LaunchConfiguration('use_ros2_control')
    world            = LaunchConfiguration('world')
    mode             = LaunchConfiguration('mode')
    map_yaml         = LaunchConfiguration('map')
    save_map         = LaunchConfiguration('save_map')

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
        arguments=['-topic', 'robot_description', '-name', 'my_bot', '-z', '0.0335'],
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
                    on_exit=[nav2_action_nav],
                )
            ),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_ros2_control', default_value='true'),
        DeclareLaunchArgument('world', default_value='empty.sdf',
                              description='World SDF file to load'),
        DeclareLaunchArgument('mode', default_value='sim_only',
                              description='sim_only | mapping | navigation'),
        DeclareLaunchArgument('map', default_value='',
                              description='Path to map YAML (required for mode:=navigation)'),
        DeclareLaunchArgument('save_map',
                              default_value='/home/cj-ubuntu/dev_ws/auto_exploration_map',
                              description='Output map path prefix for mode:=mapping'),
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
    ])
