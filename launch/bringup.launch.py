from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('rate_pubsub')
    core_launch = os.path.join(pkg_share, 'launch', 'rate.launch.py')
    tools_launch = os.path.join(pkg_share, 'launch', 'tools.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tools_launch)
        )
    ])