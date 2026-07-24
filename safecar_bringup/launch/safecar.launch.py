import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # STELLA N1 차체 기본 구동부: 모터드라이버(stella_md) + IMU(stella_ahrs) + YDLIDAR X4
    stella_bringup_launch = os.path.join(
        get_package_share_directory('stella_bringup'), 'launch', 'robot.launch.py'
    )

    return LaunchDescription([
        # 운전자 이상신호 시뮬레이션 발동 시각(초). 기본 10초 뒤 이상 발생(데모용).
        # 자유 주행/teleop 테스트 시에는 -1로 꺼야 10초 뒤 게이트가 주행을 차단하지 않는다:
        #   ros2 launch safecar_bringup safecar.launch.py anomaly_delay_sec:=-1.0
        DeclareLaunchArgument(
            'anomaly_delay_sec', default_value='10.0',
            description='N초 후 bio_anomaly=True 시뮬레이션. 0 이하면 비활성(항상 정상).'),

        # 차선 추종 자율주행 모드. true면 차선 인식 + 차선 추종 노드가 떠서
        # /cmd_vel_raw를 스스로 만든다(teleop 불필요). teleop과 동시에 켜지 말 것 —
        # 둘 다 /cmd_vel_raw에 publish해서 명령이 섞인다.
        #   ros2 launch safecar_bringup safecar.launch.py lane_follow:=true anomaly_delay_sec:=-1.0
        DeclareLaunchArgument(
            'lane_follow', default_value='false',
            description='true면 차선 인식(vision_detector) + 차선 추종(lane_follower) 실행'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stella_bringup_launch),
        ),

        # 카메라 노드. 버드아이 원근 변환(persp_src)이 해상도에 고정이므로
        # 해상도를 명시 고정한다(manual_drive.launch.py와 동일: 640x480, 상하반전 보정).
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{'width': 640, 'height': 480, 'orientation': 180}],
        ),

        # ---------------------------------------------------------------------
        # [복구됨] Hailo-8 NPU 객체 인식 노드
        # ---------------------------------------------------------------------
        Node(
             package='stella_hailo_rpi5_ros2_examples',
             executable='hailo_ros2_detection_node',
             name='hailo_ros2_detection_node',
             output='screen',
             remappings=[('image_raw', '/camera/image_raw')],
        ),
        # ---------------------------------------------------------------------

        # 차선 인식(인지부) + 차선 추종 주행(제어부) — lane_follow:=true일 때만.
        # 기존 딥러닝 기반(ufld_hailo_node)에서 OpenCV 기반(vision_detector_node)으로 변경!
        Node(
            package='safecar_perception',
            executable='vision_detector_node',
            name='vision_detector_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('lane_follow')),
            # 버드아이 원근 변환 4점(640x480 기준). 트랙에서 /perception/lane_image를
            # 보며 이 값을 조정(캘리브레이션)한다: [tl_x,tl_y, tr_x,tr_y, br_x,br_y, bl_x,bl_y]
            parameters=[{
                # 4점을 두 노란 선 위에 올린다: [tl, tr, br, bl] (실측 후 조정)
                'persp_src': [210.0, 300.0, 450.0, 300.0, 635.0, 388.0, 5.0, 395.0],
                'use_white': False,          # 노란 선만 (흰색/바닥반사 제외)
                'follow_single_line': False, # 양쪽 노란 선 → 차로 중앙 추종
            }],
        ),
        Node(
            package='safecar_control',
            executable='lane_follower_node',
            name='lane_follower_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('lane_follow')),
            # 주행 튜닝 파라미터. 트랙에서 조정(자세한 방향은 docs/RUNBOOK.md 참고).
            parameters=[{
                'cruise_speed': 0.12,
                'v_min': 0.06,
                'lookahead_dist': 0.8,
                'steer_gain': 0.8,
                'k_curv': 1.0,
                'mrm_transition_time': 2.0,
            }],
        ),

        # SafeCar 안전 감독 레이어: 제어부 + 통신부(센서 브릿지)
        Node(
            package='safecar_control',
            executable='decision_maker_node',
            name='decision_maker_node',
            output='screen',
        ),
        Node(
            package='safecar_comms',
            executable='sensor_bridge_node',
            name='sensor_bridge_node',
            output='screen',
            parameters=[{'anomaly_delay_sec': LaunchConfiguration('anomaly_delay_sec')}],
        ),
    ])