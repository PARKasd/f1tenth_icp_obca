"""Gym profile; start the external simulator before this launch."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory('obca_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('raceline_file', default_value=os.path.join(share, 'racelines', 'map.csv')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'navigation.launch.py')),
            launch_arguments={'profile_file': os.path.join(share, 'config', 'sim.yaml'),
                              'raceline_file': LaunchConfiguration('raceline_file')}.items(),
        ),
    ])
