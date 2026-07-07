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
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
        ),

        # TODO: Hailo-8 객체 인식을 붙이려면 stella_hailo_rpi5_ros2_examples의
        # hailo_ros2_detection_node를 여기에 추가하고 image_raw는 위 camera_ros가
        # publish하는 '/camera/image_raw'를 그대로 구독하면 된다 (기본 토픽명이 같아 remap 불필요).
        # 단, hailo-rpi5-examples 저장소 + install_ros2.sh 설치가 선행되어야 한다.

        # SafeCar 안전 감독 레이어: 인지부 + 제어부 + 통신부(센서 브릿지)
        Node(
            package='safecar_perception',
            executable='vision_detector_node',
            name='vision_detector_node',
            output='screen',
        ),
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
