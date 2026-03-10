# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node


# def generate_launch_description():
#     pkg_path = get_package_share_directory('cleaner_robot_fyp')
#     controllers_yaml = os.path.join(pkg_path, 'config', 'robot_controllers.yaml')
#     default_world = os.path.join(pkg_path, 'worlds', 'ws_empty.sdf')

#     # Declare optional world argument — defaults to ws_empty.sdf
#     declare_world = DeclareLaunchArgument(
#         'world',
#         default_value=default_world,
#         description='Full path to the .sdf world file to load in Gazebo'
#     )

#     world = LaunchConfiguration('world')

#     # 1. Gazebo Harmonic
#     gz_sim = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(
#                 get_package_share_directory('ros_gz_sim'),
#                 'launch',
#                 'gz_sim.launch.py'
#             )
#         ),
#         launch_arguments={'gz_args': ['-r ', world]}.items()
#     )

#     # 2. Robot State Publisher
#     rsp = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_path, 'launch', 'rsp.launch.py')
#         ),
#         launch_arguments={'use_sim_time': 'true'}.items()
#     )

#     # 3. Bridge /clock so ROS nodes get sim time
#     gz_bridge = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         arguments=[
#             '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
#             '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
#             '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
#         ],
#         output='screen'
#     )

#     # Remap Gazebo's namespaced laser frame to ROS laser_frame
#     scan_relay = Node(
#         package='topic_tools',
#         executable='relay',
#         name='scan_relay',
#         parameters=[{
#             'input_topic': '/scan',
#             'output_topic': '/scan_relayed',
#             'use_sim_time': True,
#         }],
#         output='screen'
#     )

#     # Static TF: map Gazebo's internal laser frame to our laser_frame
#     laser_tf = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='laser_tf_publisher',
#         arguments=['0', '0', '0', '0', '0', '0',
#                    'cleaner_robot/base_link/laser', 'laser_frame'],
#         output='screen'
#     )

#     # 4. Spawn robot into Gazebo
#     spawn_robot = TimerAction(
#         period=1.0,
#         actions=[Node(
#             package='ros_gz_sim',
#             executable='create',
#             arguments=['-name', 'cleaner_robot', '-topic', 'robot_description'],
#             output='screen'
#         )]
#     )

#     # 5. Joint State Broadcaster
#     spawn_joint_state_broadcaster = TimerAction(
#         period=1.0,
#         actions=[Node(
#             package='controller_manager',
#             executable='spawner',
#             arguments=[
#                 'joint_state_broadcaster',
#                 '--controller-manager', '/controller_manager',
#                 '--param-file', controllers_yaml,
#             ],
#             output='screen'
#         )]
#     )

#     # 6. Spawn diff_drive_controller
#     spawn_diff_drive = TimerAction(
#         period=1.0,
#         actions=[Node(
#             package='controller_manager',
#             executable='spawner',
#             arguments=[
#                 'diff_drive_controller',
#                 '--controller-manager', '/controller_manager',
#                 '--param-file', controllers_yaml,
#             ],
#             output='screen'
#         )]
#     )

#     return LaunchDescription([
#         declare_world,
#         gz_sim,
#         rsp,
#         gz_bridge,
#         laser_tf,
#         spawn_robot,
#         spawn_joint_state_broadcaster,
#         spawn_diff_drive,
#     ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('cleaner_robot_fyp')
    controllers_yaml = os.path.join(pkg_path, 'config', 'robot_controllers.yaml')
    default_world = os.path.join(pkg_path, 'worlds', 'ws_empty.sdf')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to the .sdf world file to load in Gazebo'
    )

    world = LaunchConfiguration('world')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items()
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        parameters=[{
            'qos_overrides./tf_static.publisher.durability': 'transient_local',  # ADD
            'use_sim_time': True,
        }],
        output='screen'
    )

    # Remap Gazebo's namespaced laser frame to ROS laser_frame
    scan_relay = Node(
        package='topic_tools',
        executable='relay',
        name='scan_relay',
        parameters=[{
            'input_topic': '/scan',
            'output_topic': '/scan_relayed',
            'use_sim_time': True,
        }],
        output='screen'
    )

    # Static TF: map Gazebo's internal laser frame to our laser_frame
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0',
                'laser_frame',                    # parent  ← SWAPPED
                'cleaner_robot/base_link/laser'], # child   ← SWAPPED
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    spawn_robot = TimerAction(
        period=.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'cleaner_robot', '-topic', 'robot_description'],
            output='screen'
        )]
    )

    spawn_joint_state_broadcaster = TimerAction(
        period=1.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/controller_manager',
                '--param-file', controllers_yaml,
            ],
            output='screen'
        )]
    )

    spawn_diff_drive = TimerAction(
        period=1.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_drive_controller',
                '--controller-manager', '/controller_manager',
                '--param-file', controllers_yaml,
            ],
            output='screen'
        )]
    )

    return LaunchDescription([
        declare_world,
        gz_sim,
        rsp,
        gz_bridge,
        laser_tf,
        spawn_robot,
        spawn_joint_state_broadcaster,
        spawn_diff_drive,
    ])