"""Hardware profile; start the sensor drivers and vehicle mux separately."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory('obca_navigation')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'navigation.launch.py')),
            launch_arguments={'profile_file': os.path.join(share, 'config', 'real.yaml')}.items(),
        ),
    ])
