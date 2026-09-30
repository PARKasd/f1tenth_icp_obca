"""Standalone manual-initial-pose ICP + OBCA stack; external LiDAR/odom/TF required."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    config = arg('params_file')
    icp_config = os.path.join(get_package_share_directory('kinematic_localization'),
                              'config', 'kinematic_localization.yaml')
    for path in (config, icp_config):
        if not os.path.isfile(path):
            raise RuntimeError(f'Required parameter file missing: {path}')
    sim = arg('use_sim_time').lower() == 'true'
    common = {'use_sim_time': sim, 'base_frame': arg('base_frame'),
              'map_frame': arg('map_frame'), 'drive_topic': arg('drive_topic'),
              'scan_topic': arg('scan_topic')}
    return [
        Node(package='kinematic_localization', executable='localization_node',
             name='kinematic_localization', output='screen',
             parameters=[icp_config, config, {
                 'use_sim_time': sim, 'base_frame': arg('base_frame'),
                 'map_frame': arg('map_frame'), 'odom_frame': arg('odom_frame'),
                 'odom_topic': arg('wheel_odom_topic'), 'lidar_topic': arg('scan_topic'),
                 'publish_map_odom_tf': arg('publish_map_odom_tf').lower() == 'true',
                 'slam_mode': True, 'require_initial_pose': True,
                 'auto_init_from_waypoints': False, 'map_name': '',
                 'smoothing_enable': False,
             }]),
        Node(package='obca_navigation', executable='planner_node', name='obca_planner',
             output='screen', parameters=[config, common]),
        Node(package='obca_navigation', executable='tracker_node', name='obca_tracker',
             output='screen', parameters=[config, common]),
    ]


def generate_launch_description():
    config = os.path.join(get_package_share_directory('obca_navigation'),
                          'config', 'navigation.yaml')
    arguments = [
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('wheel_odom_topic', default_value='/odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('publish_map_odom_tf', default_value='true'),
        DeclareLaunchArgument('drive_topic', default_value='/obca/drive',
                              description='Set to /drive for gym; use the mux autonomous input on hardware.'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_nodes)])
