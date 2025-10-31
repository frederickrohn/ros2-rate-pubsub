from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('rate_pubsub')
    param_file = os.path.join(pkg_share, 'config', 'rate_params.yaml')

    # Define the publisher node
    talker = Node(
        package='rate_pubsub',
        executable='rate_talker',
        name='rate_talker',
        output='screen',
        parameters=[param_file]
    )

    # Define the subscriber node
    listener = Node(
        package='rate_pubsub',
        executable='rate_listener',
        name='rate_listener',
        output='screen'
    )

    # Create and return the launch description
    return LaunchDescription([
        talker,
        listener
    ])