import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
  namePackage = 'cleaner_robot_fyp'
  modelFileRelativePath = 'description/robot.urdf.xacro'
  pathModelFile = os.path.join(get_package_share_directory(namePackage),
                               modelFileRelativePath)
  default_world = os.path.join(get_package_share_directory(namePackage),
                               'worlds', 'ws_empty.sdf')
  declare_world_arg = DeclareLaunchArgument(
      'world',
      default_value=default_world,
      description='Full path to .sdf world file'
  )
  world = LaunchConfiguration('world')


  clear_gz_gui_cache = ExecuteProcess(
    cmd=['bash', '-c', 'rm -f ~/.gz/sim/8/gui.config ~/.gz/sim/gui.config'],
    output='screen'
  )

  # Still process xacro for robot_state_publisher (needed for ROS TF tree / RViz)
  robotDescription = xacro.process_file(pathModelFile).toxml()

  gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
                              os.path.join(
                                get_package_share_directory('ros_gz_sim'),
                                'launch',
                                'gz_sim.launch.py'))
  gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch,
                                          launch_arguments={
                                            'gz_args': ['-r -v -v4 ', world],
                                            'on_exit_shutdown': 'true'
                                          }.items())

  # Note: No spawn node — robot model is baked directly into the SDF world file.
  # This avoids URDF->SDF conversion entirely, fixing the lidar sensor lumping issue.

  nodeRobotStatePublisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[{'robot_description': robotDescription,
                 'use_sim_time': True}]
  )

  bridge_params = os.path.join(get_package_share_directory(namePackage),
                               'config', 'bridge_parameters.yaml')
  start_gazebo_ros_bridge_cmd = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
      '--ros-args',
      '-p',
      f'config_file:={bridge_params}'
    ],
    output='screen'
  )

  # Static TF: bridges lidar_frame (URDF/ROS name) to robot/lidar_frame (Gazebo scoped name)
  # Required for RViz LaserScan display with fixed frame = base_link
  static_tf_lidar = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='lidar_tf_publisher',
    arguments=['0', '0', '0', '0', '0', '0',
               'lidar_frame',
               'robot/lidar_frame/lidar'],
    parameters=[{'use_sim_time': True}],
    output='screen'
  )

  set_lidar_topic = TimerAction(
    period=5.0,
    actions=[
      ExecuteProcess(
        cmd=['gz', 'topic', '-t', '/gui/visualize_lidar',
            '-m', 'gz.msgs.StringMsg',
            '-p', 'data: "/scan"'],
        output='screen'
      )
    ]
  )

  # set_lidar_topic = TimerAction(
  #   period=3.0,   # initial wait before first attempt
  #   actions=[
  #     ExecuteProcess(
  #       cmd=[
  #         'bash', '-c',
  #         'until gz topic -l | grep -q "^/scan$"; do sleep 2; done; '
  #         'for i in $(seq 1 10); do '
  #         '  gz topic -t /gui/visualize_lidar -m gz.msgs.StringMsg -p \'data: "/scan"\'; '
  #         '  sleep 1; '
  #         'done'
  #       ],
  #       output='screen'
  #     )
  #   ]
  # )
  
  # script_path = os.path.join(get_package_share_directory(namePackage),
  #                           'launch', 'init_lidar_viz.sh')
  # set_lidar_topic = TimerAction(
  #   period=3.0,   # initial wait before first attempt
  #   actions=[
  #     ExecuteProcess(
  #       cmd=['bash', script_path],
  #       output='screen'
  #     )
  #   ]
  # )

  launchDescriptionObject = LaunchDescription()

  launchDescriptionObject.add_action(declare_world_arg)
  launchDescriptionObject.add_action(clear_gz_gui_cache)
  launchDescriptionObject.add_action(gazeboLaunch)
  launchDescriptionObject.add_action(nodeRobotStatePublisher)
  launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
  launchDescriptionObject.add_action(static_tf_lidar)
  launchDescriptionObject.add_action(set_lidar_topic)
  return launchDescriptionObject