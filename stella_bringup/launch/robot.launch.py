#!/usr/bin/env python3
#
# Copyright 2025 NTREX CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# 이 프로젝트 하드웨어 구성(N1 차체 + YDLIDAR X4 단일 라이다)에 맞춰
# RealSense/웹캠/포인트클라우드/Hailo 분기를 모두 제거하고 단순화한 버전.
# 카메라/Hailo 인식은 이 launch가 아니라 safecar_bringup에서 별도로 담당한다.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():
    md_pkg_dir = LaunchConfiguration(
        'md_pkg_dir',
        default=os.path.join(get_package_share_directory('stella_md'), 'launch'))

    ahrs_pkg_dir = LaunchConfiguration(
        'ahrs_pkg_dir',
        default=os.path.join(get_package_share_directory('stella_ahrs'), 'launch'))

    ydlidar_pkg_dir = LaunchConfiguration(
        'ydlidar_pkg_dir',
        default=os.path.join(get_package_share_directory('ydlidar'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/stella_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # 모터드라이버
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([md_pkg_dir, '/stella_md_launch.py']),
        ),

        # IMU/AHRS
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ahrs_pkg_dir, '/stella_ahrs_launch.py']),
        ),

        # YDLIDAR X4 (단일 라이다, /scan publish)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ydlidar_pkg_dir, '/ydlidar_launch.py']),
        ),
    ])
