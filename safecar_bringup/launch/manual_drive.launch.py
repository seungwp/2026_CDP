import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """수동(키보드) 주행 + 촬영 전용 구성.

    safecar.launch.py와 달리 안전 게이트(decision_maker)·Hailo·차선 노드를
    전혀 띄우지 않는다. 차체 구동부(stella base)와 카메라만 올린다.

    - 게이트가 없으므로 teleop은 '/cmd_vel'로 직접 publish하면 된다
      (remap 불필요). stella_md가 바로 받아 구동한다.
      → '/cmd_vel_raw 1.0초 끊김 → 정지' 같은 스톱-고 현상이 없다.
    - 카메라(/camera/image_raw)가 떠 있으므로 record_drive.py로 녹화 가능.

    실행 예:
        ros2 launch safecar_bringup manual_drive.launch.py
        # 터미널 2: python3 ~/teleop.py          (/cmd_vel 로 직접 조종)
        # 터미널 3: ~/record_drive.sh            (주행 영상 녹화)

    주의: 안전 게이트가 없어 장애물 자동정지/명령 타임아웃이 동작하지 않는다.
    조종을 멈추려면 teleop에서 's'(정지) 또는 'q'(종료, 정지 명령 발행)를 쓸 것.
    """
    stella_bringup_launch = os.path.join(
        get_package_share_directory('stella_bringup'), 'launch', 'robot.launch.py'
    )

    return LaunchDescription([
        # STELLA N1 차체 구동부: 모터드라이버(stella_md) + IMU(stella_ahrs) + YDLIDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stella_bringup_launch),
        ),

        # 라즈베리파이 CSI 카메라. '/camera/image_raw' publish (640x480, 상하반전 보정).
        # 값의 근거는 safecar.launch.py 주석 참고.
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{'width': 640, 'height': 480, 'orientation': 180}],
        ),
    ])
