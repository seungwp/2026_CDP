from glob import glob

from setuptools import find_packages, setup

package_name = 'safecar'

# 인지(perception) · 제어(control) · 통신(comms)은 하위 모듈로 나눠 담당 영역을 구분한다.
# 예전에는 담당자별로 ROS 패키지를 따로 뒀지만(코드 1천 줄에 패키지 5개),
# 하나로 합쳐 설정 파일과 실행 경로를 단순화했다.
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seungje Kim',
    maintainer_email='kimseungwp@yu.ac.kr',
    description='SafeCar 안전 감독 레이어 (인지·제어·통신 + 통합 launch)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vision_detector_node = safecar.perception.vision_detector_node:main',
            'decision_maker_node = safecar.control.decision_maker_node:main',
            'lane_follower_node = safecar.control.lane_follower_node:main',
            'sensor_bridge_node = safecar.comms.sensor_bridge_node:main',
        ],
    },
)
