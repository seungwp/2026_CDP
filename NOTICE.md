# NOTICE

이 워크스페이스의 `stella/`, `stella_bringup/`, `stella_description/`,
`stella_hailo_rpi5_ros2_examples/`, `stella_teleop_bluetooth/`는
[NTREX CO., LTD.](https://github.com/ntrexlab)의 STELLA 로봇 플랫폼 ROS2 패키지를
기반으로 한다.

- 원본: https://github.com/ntrexlab/STELLA_N5_ROS2 (Apache License 2.0 / MIT, 패키지별 상이)
- YDLIDAR X4 이식 및 N1 차체용 모터 기구학 수정: 팀원 fork (https://github.com/seungwp/cdp)에서 선행 작업
- 이 워크스페이스에서 추가로: RealSense/SLAMTEC 라이다/포인트클라우드 등 미사용 패키지 제거,
  `stella_bringup`의 launch를 조건 분기 없는 단일 구성으로 단순화

각 패키지의 라이선스/저작권 표시는 해당 패키지 소스 파일 상단 주석 및 `package.xml`을 따른다.

`safecar/`, `safecar_bringup/`은 이 프로젝트에서 새로 작성한 코드다.
