from setuptools import find_packages, setup

package_name = 'safecar_control'

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
    description='SafeCar 제어부: 센서 상태를 종합해 주행 명령을 결정',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'decision_maker_node = safecar_control.decision_maker_node:main',
            'lane_follower_node = safecar_control.lane_follower_node:main',
        ],
    },
)
