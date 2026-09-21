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
    # (safecar_start.sh도 이 launch를 그대로 쓴다 — 실행 경로는 이것 하나다)
    stella_bringup_launch = os.path.join(
        get_package_share_directory('stella_bringup'), 'launch', 'robot.launch.py'
    )

    return LaunchDescription([
        # 운전자 이상신호 시뮬레이션 발동 시각(초).
        # 실제 감지는 노트북 VM의 driver_monitor_node(cdp-remotepc 레포, 정수영)가 담당하므로
        # 기본값은 비활성(-1.0)이다 — 켜두면 같은 토픽을 두 곳에서 발행해 신호가 싸운다.
        # VM 없이 Pi 단독으로 갓길 대피(MRM)를 데모할 때만 켠다:
        #   ros2 launch safecar safecar.launch.py lane_follow:=true anomaly_delay_sec:=10.0
        DeclareLaunchArgument(
            'anomaly_delay_sec', default_value='-1.0',
            description='N초 후 bio_anomaly=True 시뮬레이션. 0 이하면 비활성(항상 정상).'),

        # 차선 추종 자율주행 모드. true면 차선 추종 노드가 떠서
        # /cmd_vel_raw를 스스로 만든다(teleop 불필요). teleop과 동시에 켜지 말 것 —
        # 둘 다 /cmd_vel_raw에 publish해서 명령이 섞인다.
        #   ros2 launch safecar safecar.launch.py lane_follow:=true
        DeclareLaunchArgument(
            'lane_follow', default_value='false',
            description='true면 차선 추종(lane_follower) 실행 = 실제로 주행한다'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stella_bringup_launch),
        ),

        # 라즈베리파이 카메라 모듈(CSI/libcamera) 드라이버. '/camera/image_raw' publish.
        # 별도 설치 필요: https://github.com/christianrauch/camera_ros
        # 640x480 고정: hailo_ros2_detection_node의 GStreamer 파이프라인이 640x480을
        # 가정하므로, 기본값(800x600)으로 두면 추론 입력 영상이 깨진다(4분할 증상).
        # orientation 180: 카메라 모듈이 차체에 거꾸로 장착되어 있어 센서 수준에서 뒤집음
        # (libcamera 처리라 CPU 비용 없음). 장착 방향이 바뀌면 이 값만 수정.
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{'width': 640, 'height': 480, 'orientation': 180}],
        ),

        # Hailo-8 NPU 객체 인식. '/detection_image'(디버그용 박스 영상)와
        # '/perception/obstacle_detected'(장애물 유무, 제어부 입력) publish.
        # 선행 조건: hailo-rpi5-examples(hailo_apps_infra) 설치 (Pi에 설치 완료됨).
        # remap 필요: 노드는 상대 토픽 'image_raw'를 구독하므로 camera_ros의
        # '/camera/image_raw'로 연결해줘야 한다.
        Node(
            package='stella_hailo_rpi5_ros2_examples',
            executable='hailo_ros2_detection_node',
            name='hailo_ros2_detection_node',
            output='screen',
            remappings=[('image_raw', '/camera/image_raw')],
        ),

        # 차선 인식은 항상 켠다 — 주행 전에도 /perception/lane_image로 차선이 잡히는지
        # 확인할 수 있어야 한다. 실제 주행(lane_follower)만 lane_follow:=true로 켠다.
        # 장애물 감지는 위 Hailo 노드가 전담한다.
        Node(
            package='safecar',
            executable='vision_detector_node',
            name='vision_detector_node',
            output='screen',
        ),
        Node(
            package='safecar',
            executable='lane_follower_node',
            name='lane_follower_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('lane_follow')),
        ),

        # SafeCar 안전 감독 레이어: 제어부 + 통신부(센서 브릿지)
        Node(
            package='safecar',
            executable='decision_maker_node',
            name='decision_maker_node',
            output='screen',
        ),
        Node(
            package='safecar',
            executable='sensor_bridge_node',
            name='sensor_bridge_node',
            output='screen',
            parameters=[{'anomaly_delay_sec': LaunchConfiguration('anomaly_delay_sec')}],
        ),
    ])
