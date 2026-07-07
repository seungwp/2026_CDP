# 대시보드 (Dashboard)

로컬 프로세스로 붙일지, 원격 PC/모바일에서 네트워크로 붙일지 아직 미정.

ROS2로 전환했기 때문에, 방식이 정해지면 어느 쪽이든 **구독할 토픽만 알면 됨**:
- `/control/driving_state` (std_msgs/String) — 현재 주행 상태 (`NORMAL` / `EMERGENCY_BRAKE` / `MRM_PULL_OVER`)
- `/perception/obstacle_detected` (std_msgs/Bool)
- `/sensors/bio_anomaly` (std_msgs/Bool)
- `/scan` (sensor_msgs/LaserScan, ydlidar_ros) — 라이다
- `/imu/yaw` (std_msgs/Float64, stella_ahrs) — IMU
- `/odom` (nav_msgs/Odometry, stella_md) — 주행 오도메트리

**로컬**이면: 같은 워크스페이스에 이 패키지를 rclpy 노드로 구현해서 위 토픽들을 그대로 subscribe.
**원격**이면: ROS2 DDS가 네트워크를 기본 지원하므로, 원격 PC에 동일하게 ROS2를 설치하고 같은 ROS_DOMAIN_ID로 맞추면 별도 통신 계층 구현 없이 위 토픽을 그대로 구독 가능 (`STELLA_N1_REMOTEPC_X4_ROS2_v2.0` 레퍼런스와 동일한 패턴).
