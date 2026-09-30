import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # launch_ros는 존재하지 않는 parameter file을 조용히 건너뛸 수 있다. 그러면 노드는
    # gate/source downsample/smoothing이 코드 기본값인 채 정상 기동하므로, launch 단계에서
    # 깨진 symlink까지 포함해 즉시 차단한다. 파일 내용의 세대 일치는 노드의
    # config_schema_version 가드가 한 번 더 확인한다.
    config = os.path.join(
        get_package_share_directory('kinematic_localization'),
        'config',
        'kinematic_localization.yaml',
    )
    if not os.path.isfile(config):
        raise RuntimeError(
            f'필수 파라미터 파일이 없거나 symlink가 깨졌습니다: {config}\n'
            'src/kinematic_localization/config와 install overlay를 복구·재빌드한 뒤 '
            '다시 실행하세요.')
    map_name = LaunchConfiguration('map_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_mode = LaunchConfiguration('slam_mode')
    map_output_file = LaunchConfiguration('map_output_file')
    fast_corner_free_mode = LaunchConfiguration('fast_corner_free_mode')
    corner_yaw_rate_thresh = LaunchConfiguration('corner_yaw_rate_thresh')
    corner_speed_thresh = LaunchConfiguration('corner_speed_thresh')
    corner_lat_accel_thresh = LaunchConfiguration('corner_lat_accel_thresh')
    odom_window_at_scan_end = LaunchConfiguration('odom_window_at_scan_end')
    scan_stamp_convention = LaunchConfiguration('scan_stamp_convention')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='',
            description='Frozen map name: maps/<map_name>.kissmap in the package share '
                        '(an absolute path is used as-is, empty = pure odometry). '
                        'Ignored when slam_mode is true.'),
        DeclareLaunchArgument(
            'slam_mode',
            default_value='false',
            description='Online SLAM mode: no frozen map, auto-init at identity on the '
                        'first scan, save the accumulated map via the ~/save_map service '
                        'or on shutdown.'),
        DeclareLaunchArgument(
            'map_output_file',
            default_value='slam_map.kissmap',
            description='Output .kissmap path for slam_mode (relative paths resolve '
                        'against the process cwd).'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # §8 fast-corner free mode. YAML을 고치지 않고 실차에서 A/B 하려고 인자로 뺐다.
        # 파라미터는 생성자에서 한 번만 읽으므로 `ros2 param set`으로는 못 바꾼다 —
        # 반드시 기동 시점에 넘겨야 한다.
        DeclareLaunchArgument(
            'fast_corner_free_mode', default_value='false',
            description='고속 코너 프레임에서 Omega/횡하한/반복상한·수렴/게이트/스무딩을 '
                        '모두 우회한다. 실차 미검증 — 기본 false.'),
        DeclareLaunchArgument('corner_yaw_rate_thresh', default_value='0.8'),
        DeclareLaunchArgument('corner_speed_thresh', default_value='3.0'),
        DeclareLaunchArgument('corner_lat_accel_thresh', default_value='6.0'),
        # §9 스캔 끝-시각 동기화. 2026-08-22 실차 A/B/A 로 검증돼 기본 true 가 됐다
        # (자세 보정 0.83 -> 0.43 -> 0.87 deg, 양쪽에서 돌아옴 = -49%).
        # 되돌려서 비교하려면 odom_window_at_scan_end:=false 로 띄운다.
        DeclareLaunchArgument(
            'odom_window_at_scan_end', default_value='true',
            description='등록 prior 와 발행 스탬프를 스캔 스윕 끝에 맞춘다 (OdomAtStrict + '
                        '스캔 FIFO). 실차 A/B/A 검증 완료 — 되돌리려면 false.'),
        DeclareLaunchArgument(
            'scan_stamp_convention', default_value='begin',
            description="/scan 헤더가 스윕의 'begin'(이 차, 재생 A/B 로 판정) 인지 'end' 인지. "
                        'begin 이면 scan_end = header + (N-1)*time_increment.'),
        Node(
            package='kinematic_localization',
            executable='localization_node',
            name='kinematic_localization',
            output='screen',
            parameters=[config, {
                'map_name': map_name,
                'slam_mode': slam_mode,
                'map_output_file': map_output_file,
                'use_sim_time': use_sim_time,
                'fast_corner_free_mode': fast_corner_free_mode,
                'corner_yaw_rate_thresh': corner_yaw_rate_thresh,
                'corner_speed_thresh': corner_speed_thresh,
                'corner_lat_accel_thresh': corner_lat_accel_thresh,
                'odom_window_at_scan_end': ParameterValue(
                    odom_window_at_scan_end, value_type=bool),
                'scan_stamp_convention': ParameterValue(
                    scan_stamp_convention, value_type=str),
            }],
        ),
    ])
