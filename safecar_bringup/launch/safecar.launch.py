import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # STELLA N1 차체 기본 구동부: 모터드라이버(stella_md) + IMU(stella_ahrs) + YDLIDAR X4
    stella_bringup_launch = os.path.join(
        get_package_share_directory('stella_bringup'), 'launch', 'robot.launch.py'
    )

    return LaunchDescription([
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
        # 선행 조건: hailo-rpi5-examples + install_ros2.sh 설치 (Pi에 설치 완료됨).
        # remap 필요: 노드는 상대 토픽 'image_raw'를 구독하므로 camera_ros의
        # '/camera/image_raw'로 연결해줘야 한다.
        Node(
            package='stella_hailo_rpi5_ros2_examples',
            executable='hailo_ros2_detection_node',
            name='hailo_ros2_detection_node',
            output='screen',
            remappings=[('image_raw', '/camera/image_raw')],
        ),

        # SafeCar 안전 감독 레이어: 제어부 + 통신부(센서 브릿지)
        # NOTE: safecar_perception의 vision_detector_node는 잠정 제외 —
        # 장애물 감지는 위 Hailo 노드가 실추론으로 담당하게 되어, 남은 역할이던
        # 시간 기반 임시 장애물 신호가 오히려 실신호와 충돌한다. 차선 인식 결과를
        # publish하게 되면(인지부 작업) 그때 다시 추가할 것.
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
        ),
    ])
