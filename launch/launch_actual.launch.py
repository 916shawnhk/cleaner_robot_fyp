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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                             IncludeLaunchDescription, RegisterEventHandler,
                             TimerAction)
from launch.conditions import LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """
    launch_actual.launch.py

      mode:=robot_only  (default) — Hardware + sensors only. No change.
      mode:=mapping               — Hardware + slam_toolbox + nav2 + WFE.
      mode:=navigation            — Hardware + amcl + nav2. Requires map:=.
    """

    package_name = 'cleaner_robot_fyp'
    pkg_share    = get_package_share_directory(package_name)

    mode     = LaunchConfiguration('mode')
    map_yaml = LaunchConfiguration('map')

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
                    on_exit=[nav2_navigation],
                )
            ),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='robot_only',
                              description='robot_only | mapping | navigation'),
        DeclareLaunchArgument('map', default_value='',
                              description='Path to map YAML (required for mode:=navigation)'),
        rsp,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        twist_mux,
        lidar,
        camera,
        mapping_group,
        navigation_group,
    ])