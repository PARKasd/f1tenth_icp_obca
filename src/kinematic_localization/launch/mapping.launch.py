import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # localization launch와 같은 E2 fail-fast 계약. 없는 파일을 parameters=[...]에 넘겨
    # 코드 기본값으로 매핑 세션을 시작하지 않는다.
    config = os.path.join(
        get_package_share_directory('kinematic_localization'), 'config', 'mapping.yaml')
    if not os.path.isfile(config):
        raise RuntimeError(
            f'필수 파라미터 파일이 없거나 symlink가 깨졌습니다: {config}\n'
            'src/kinematic_localization/config와 install overlay를 복구·재빌드한 뒤 '
            '다시 실행하세요.')
    bag_path = LaunchConfiguration('bag_path')
    output_path = LaunchConfiguration('output_path')
    lidar_topic = LaunchConfiguration('lidar_topic')

    return LaunchDescription([
        DeclareLaunchArgument('bag_path', description='Input rosbag2 directory'),
        DeclareLaunchArgument(
            'output_path', default_value='map.kissmap',
            description='Output .kissmap file path'),
        DeclareLaunchArgument('lidar_topic', default_value='/scan'),
        Node(
            package='kinematic_localization',
            executable='mapping_node',
            name='kinematic_mapping',
            output='screen',
            parameters=[config, {
                'bag_path': bag_path,
                'output_path': output_path,
                'lidar_topic': lidar_topic,
            }],
        ),
    ])
