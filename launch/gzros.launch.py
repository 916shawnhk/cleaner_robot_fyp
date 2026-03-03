# import os

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from launch.actions import TimerAction

# import xacro 

# def generate_launch_description():

#   pkg_path = get_package_share_directory('cleaner_robot_fyp')

#   world_file = os.path.join(pkg_path, 'worlds', 'ws_empty.sdf')

#   ros2_control_params = os.path.join(pkg_path, 'config', 'robot_controllers.yaml')

#   xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
#   robot_description_config = xacro.process_file(xacro_file)

#   return LaunchDescription([

#     # Launch Gazebo Harmonic
#     IncludeLaunchDescription(
#       PythonLaunchDescriptionSource(
#         os.path.join(
#         get_package_share_directory('ros_gz_sim'),
#         'launch',
#         'gz_sim.launch.py'
#         )
#       ),
#       launch_arguments={'gz_args': '-r ' + world_file}.items(),
#     ),

#     # Launch robot_state_publisher
#     Node(
#       package='robot_state_publisher',
#       executable='robot_state_publisher',
#       output='screen',
#       parameters=[{
#         'robot_description': robot_description_config.toxml(),
#         'use_sim_time': True,
#         'ros2_control': ros2_control_params
#       }]
#     ),

#     # Spawn robot into Gazebo
#     Node(
#       package='ros_gz_sim',
#       executable='create',
#       arguments=[
#         '-name', 'cleaner_robot',
#         '-topic', 'robot_description'
#       ],
#       output='screen'
#     ),

#     # Spawn diff drive controller
#     Node(
#       package='controller_manager',
#       executable='spawner',
#       arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
#       output='screen'
#     ),
#     Node(
#       package='controller_manager',
#       executable='ros2_control_node',
#       parameters=[
#         ros2_control_params,
#         {'use_sim_time': True}
#       ],
#       output='screen'
#     )

#   ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('cleaner_robot_fyp')
    world_file = os.path.join(pkg_path, 'worlds', 'ws_empty.sdf')
    controllers_yaml = os.path.join(pkg_path, 'config', 'robot_controllers.yaml')

    # 1. Gazebo Harmonic
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': '-r ' + world_file}.items()
    )

    # 2. Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 3. Bridge /clock so ROS nodes get sim time
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 4. Spawn robot into Gazebo (delayed)
    spawn_robot = TimerAction(
        period=1.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'cleaner_robot', '-topic', 'robot_description'],
            output='screen'
        )]
    )

    # 5. Spawn diff_drive_controller (delayed)
    #    --param-file passes the YAML so controller_manager knows the controller type
    spawn_diff_drive = TimerAction(
        period=2.0,
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
        gz_sim,
        rsp,
        gz_bridge,
        spawn_robot,
        spawn_diff_drive,
    ])