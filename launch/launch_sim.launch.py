import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
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

  # Toggle: 'true' = ros2_control, 'false' = Gazebo built-in DiffDrive plugin
  declare_use_ros2_control_arg = DeclareLaunchArgument(
      'use_ros2_control',
      default_value='true',
      description='Use ros2_control if true, Gazebo DiffDrive plugin if false'
  )
  use_ros2_control = LaunchConfiguration('use_ros2_control')

  # remove config cache
  clear_gz_gui_cache = ExecuteProcess(
    cmd=['bash', '-c', 'rm -f ~/.gz/sim/8/gui.config ~/.gz/sim/gui.config'],
    output='screen'
  )

  # Note: xacro mappings must be resolved at launch-time.
  # use_ros2_control LaunchConfiguration is a string ('true'/'false') at this point.
  # We extract it by evaluating the perform() equivalent via a workaround:
  # Since launch_sim.launch.py processes xacro eagerly with the Python module,
  # we read the launch argument default here. To support runtime toggling,
  # use_ros2_control is passed as a xacro mapping using perform_substitutions.
  import sys
  use_ros2_control_val = 'true'
  sim_mode_val = 'true'
  for arg in sys.argv:
      if arg.startswith('use_ros2_control:='):
          use_ros2_control_val = arg.split(':=')[1]

  robotDescription = xacro.process_file(
      pathModelFile,
      mappings={'use_ros2_control': use_ros2_control_val,
                'sim_mode': 'true'}
  ).toxml()
  
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

  combined_spawner = TimerAction(
    period=15.0,
    actions=[
      Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
          "joint_broad",
          "diff_cont",
          '--controller-ros-args',
          '-r /diff_cont/cmd_vel:=/cmd_vel'
        ],
        condition=IfCondition(use_ros2_control),
        output='screen',
      )
    ],
    condition=IfCondition(use_ros2_control),
  )

   # twist_mux: handle multiple control sources
  twist_mux_config = os.path.join(get_package_share_directory(namePackage),
                                        'config', 'twist_mux.yaml')
  twist_mux = Node(
      package='twist_mux',
      executable='twist_mux',
      output='screen',
      remappings={('/cmd_vel_out', '/cmd_vel')},
      parameters=[
          {'use_sim_time': True},
          twist_mux_config])

  launchDescriptionObject = LaunchDescription()

  launchDescriptionObject.add_action(declare_world_arg)
  launchDescriptionObject.add_action(declare_use_ros2_control_arg)
  launchDescriptionObject.add_action(clear_gz_gui_cache)
  launchDescriptionObject.add_action(gazeboLaunch)

  launchDescriptionObject.add_action(spawnModelNodeGazebo)
  launchDescriptionObject.add_action(nodeRobotStatePublisher)
  launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
  launchDescriptionObject.add_action(static_tf_lidar) # RVIZ CHANGE
  launchDescriptionObject.add_action(static_tf_camera)   # CAMERA CHANGE
  launchDescriptionObject.add_action(combined_spawner)
  launchDescriptionObject.add_action(twist_mux)

  return launchDescriptionObject
