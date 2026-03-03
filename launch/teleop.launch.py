from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Opens teleop in its own xterm window (fixes keyboard focus issue too)
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix='xterm -e',
        remappings=[('/cmd_vel', '/teleop/cmd_vel')]
    )

    # Converts Twist (from teleop) -> TwistStamped (for diff_drive_controller)
    relay = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        parameters=[{'use_sim_time': True, 'frame_id': ''}],
        remappings=[
            ('/cmd_vel_in', '/teleop/cmd_vel'),
            ('/cmd_vel_out', '/diff_drive_controller/cmd_vel'),
        ]
    )

    return LaunchDescription([
        teleop,
        relay,
    ])