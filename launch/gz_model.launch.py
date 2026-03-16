import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
  robotXacroName = 'robot'
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

  # remove config cache
  clear_gz_gui_cache = ExecuteProcess(
    cmd=['bash', '-c', 'rm -f ~/.gz/sim/8/gui.config ~/.gz/sim/gui.config'],
    output='screen'
  )

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

  spawnModelNodeGazebo = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
      '-name', robotXacroName,
      '-topic', 'robot_description',
      '-z', '0.0335',
    ],
    output='screen',
  )

  nodeRobotStatePublisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[{'robot_description': robotDescription,
                 'use_sim_time':True}]
  )

  bridge_params = os.path.join(get_package_share_directory(namePackage),
                               'config','bridge_parameters.yaml')
  start_gazebo_ros_bridge_cmd = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
      '--ros-args',
      '-p',
      f'config_file:={bridge_params}'
    ],
    # # ADD parameters to the parameter_bridge node:
    # parameters=[{
    #   'qos_overrides./tf_static.publisher.durability': 'transient_local',
    # }],
    output='screen'
  )

  # RVIZ CHANGE: lidar
  static_tf_lidar = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='lidar_tf_publisher',
    arguments=['0', '0', '0', '0', '0', '0',
               'lidar_frame',
               'robot/base_link/lidar'
              ],
    parameters=[{'use_sim_time': True}],
    output='screen'
  )

  # camera
  static_tf_camera = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='camera_tf_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 
               'robot/base_link/camera_link',
               'camera_optical_link'
              ],
    parameters=[{'use_sim_time': True}],
    output='screen'
  )

  # ros2_control
  diff_drive_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "diff_cont",
        '--controller-ros-args',
        '-r /diff_cont/cmd_vel:=/cmd_vel'
    ],
  )

  joint_broad_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_broad"],
  )

  launchDescriptionObject = LaunchDescription()

  launchDescriptionObject.add_action(declare_world_arg)
  launchDescriptionObject.add_action(clear_gz_gui_cache)
  launchDescriptionObject.add_action(gazeboLaunch)

  launchDescriptionObject.add_action(spawnModelNodeGazebo)
  launchDescriptionObject.add_action(nodeRobotStatePublisher)
  launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
  launchDescriptionObject.add_action(static_tf_lidar) # RVIZ CHANGE
  launchDescriptionObject.add_action(static_tf_camera)   # CAMERA CHANGE
  launchDescriptionObject.add_action(diff_drive_spawner)
  launchDescriptionObject.add_action(joint_broad_spawner)

  return launchDescriptionObject
