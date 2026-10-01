"""실행 중인 SLAM 내비게이션의 관측 지도·차량·경로를 RViz에 표시합니다."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _viewer(context):
    config = LaunchConfiguration('rviz_config').perform(context)
    if not os.path.isfile(config):
        raise RuntimeError(f'RViz 설정 파일을 찾을 수 없습니다: {config}')
    return [Node(package='rviz2', executable='rviz2', name='obca_rviz',
                 arguments=['-d', config], output='screen')]


def generate_launch_description():
    config = os.path.join(get_package_share_directory('obca_navigation'),
                          'config', 'navigation.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=config),
        OpaqueFunction(function=_viewer),
    ])
