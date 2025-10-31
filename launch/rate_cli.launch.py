from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare the launch argument for rate with a default value of 10
    hz_arg = DeclareLaunchArgument(
        'publish_hz',
        default_value='10.0',
        description='Publishing rate in Hz'
    )

    # Get the launch configuration for rate
    publish_hz = LaunchConfiguration('publish_hz')

    # Define the publisher node
    talker = Node(
        package='rate_pubsub',
        executable='rate_talker',
        name='rate_talker',
        output='screen',
        parameters=[{'publish_hz': LaunchConfiguration('publish_hz')}]
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
        hz_arg,
        talker,
        listener
    ])