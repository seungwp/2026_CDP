from setuptools import find_packages, setup

package_name = 'safecar_comms'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@todo.todo',
    description='SafeCar 통신부: ESP32/STM32 등 외부 센서 보드를 ROS2 토픽으로 중계',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_bridge_node = safecar_comms.sensor_bridge_node:main',
        ],
    },
)
