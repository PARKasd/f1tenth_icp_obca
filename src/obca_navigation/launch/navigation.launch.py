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
    profile = arg('profile_file')
    icp_config = os.path.join(get_package_share_directory('kinematic_localization'),
                              'config', 'kinematic_localization.yaml')
    for path in (config, icp_config, *([profile] if profile else [])):
        if not os.path.isfile(path):
            raise RuntimeError(f'Required parameter file missing: {path}')
    # Empty CLI values preserve YAML. Explicit CLI overrides win over both files.
    common = {name: arg(name) for name in
              ('base_frame', 'map_frame', 'drive_topic', 'scan_topic') if arg(name)}
    if arg('use_sim_time'):
        if arg('use_sim_time').lower() not in ('true', 'false'):
            raise ValueError('use_sim_time must be true or false')
        common['use_sim_time'] = arg('use_sim_time').lower() == 'true'
    icp_overrides = {name: common[name] for name in
                     ('use_sim_time', 'base_frame', 'map_frame') if name in common}
    for launch_name, param_name in (('odom_frame', 'odom_frame'),
                                    ('wheel_odom_topic', 'odom_topic'),
                                    ('scan_topic', 'lidar_topic')):
        if arg(launch_name):
            icp_overrides[param_name] = arg(launch_name)
    if arg('publish_map_odom_tf'):
        if arg('publish_map_odom_tf').lower() not in ('true', 'false'):
            raise ValueError('publish_map_odom_tf must be true or false')
        icp_overrides['publish_map_odom_tf'] = arg('publish_map_odom_tf').lower() == 'true'
    icp_overrides.update(slam_mode=True, require_initial_pose=True,
                         auto_init_from_waypoints=False, map_name='', smoothing_enable=False)
    parameters = [config] + ([profile] if profile else [])
    return [
        Node(package='kinematic_localization', executable='localization_node',
             name='kinematic_localization', output='screen',
             parameters=[icp_config, *parameters, icp_overrides]),
        Node(package='obca_navigation', executable='planner_node', name='obca_planner',
             output='screen', parameters=[*parameters, common]),
        Node(package='obca_navigation', executable='tracker_node', name='obca_tracker',
             output='screen', parameters=[*parameters, common]),
    ]


def generate_launch_description():
    config = os.path.join(get_package_share_directory('obca_navigation'),
                          'config', 'navigation.yaml')
    arguments = [
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('profile_file', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value=''),
        DeclareLaunchArgument('scan_topic', default_value=''),
        DeclareLaunchArgument('wheel_odom_topic', default_value=''),
        DeclareLaunchArgument('base_frame', default_value=''),
        DeclareLaunchArgument('odom_frame', default_value=''),
        DeclareLaunchArgument('map_frame', default_value=''),
        DeclareLaunchArgument('publish_map_odom_tf', default_value=''),
        DeclareLaunchArgument('drive_topic', default_value='',
                              description='Set to /drive for gym; use the mux autonomous input on hardware.'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_nodes)])
