from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # rqt_plot preloaded on /heartbeat
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            arguments=['/heartbeat'],
            output='screen'
        ),
    ])