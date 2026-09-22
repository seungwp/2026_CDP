import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo, LaserScan

from safecar.protocol import COMMAND_NORMAL
from safecar.control.decision_maker import DecisionMaker, judge_obstacle
from safecar.control.scan_sectors import sector_min


class DecisionMakerNode(Node):
    """비전/생체신호 토픽을 구독해 주행 상태를 판단하고, /cmd_vel의 단일 게이트로 동작한다. (제어부)

    안전 게이트 구조: 주행 명령(teleop, 추후 차선 추종 노드)은 '/cmd_vel_raw'로 들어오고,
    '/cmd_vel'은 이 노드만 publish한다 — 비상 시 정지 명령이 주행 명령과 경쟁하지 않는다.
    - NORMAL: 신선한(cmd_vel_timeout 이내) /cmd_vel_raw를 10Hz로 통과시킨다.
    - 비상(EMERGENCY_BRAKE): 주행 명령을 차단하고 정지로 오버라이드.
      장애물 판단은 Hailo(무엇인지) + 라이다(얼마나 가까운지) 퓨전이고,
      카메라·Hailo가 끊기면 정지한다(fail-safe). → decision_maker.judge_obstacle
    - MRM_PULL_OVER: lane_follower가 자체적으로 갓길 주행 후 정차하므로 명령을 통과시킨다.
    """

    def __init__(self):
        super().__init__('decision_maker_node')
        self.decision_maker = DecisionMaker()

        self.declare_parameter('cmd_vel_timeout', 1.0)
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        # 운전자 이상 래치. UN R157: MRM으로 정차한 차량은 수동 입력 없이 다시 움직여선 안 된다.
        # 래치가 없으면 이상신호가 한 프레임만 False로 튀어도 차가 갓길에서 재출발한다.
        # 해제하려면 노드를 다시 띄워야 한다(주행 튜닝 중에는 false로 꺼서 쓸 것).
        self.declare_parameter('bio_latch', True)
        self.bio_latch = self.get_parameter('bio_latch').value

        # --- 장애물 판단: 카메라(Hailo) + 라이다 퓨전 (decision_maker.judge_obstacle) ---
        # 인지 입력이 이 시간 넘게 끊기면 정지. 카메라는 camera_info로 본다 —
        # Hailo 노드는 마지막 판정을 타이머로 재발행해서 카메라가 죽어도 False가 계속 나온다.
        self.declare_parameter('perception_timeout', 1.0)
        # 벤치에서 카메라/Hailo 없이 게이트만 시험할 때 false로 끈다.
        self.declare_parameter('require_perception', True)
        # 라이다 퓨전. false면 예전처럼 Hailo 단독 판단.
        self.declare_parameter('fuse_lidar', True)
        # 전방 섹터 중심각/반각. 라이다 reversion 설정 때문에 0°가 실제 전방인지
        # scan_check.py로 실측할 것 (docs/RUNBOOK.md 3-4).
        self.declare_parameter('obstacle_front_deg', 0.0)
        self.declare_parameter('obstacle_front_half_deg', 20.0)
        self.declare_parameter('obstacle_confirm_m', 1.0)   # Hailo 감지를 인정하는 전방 거리
        self.declare_parameter('emergency_stop_m', 0.3)     # 종류 무관 즉시 정지 거리
        self.declare_parameter('scan_timeout', 1.0)
        for name in ('perception_timeout', 'require_perception', 'fuse_lidar',
                     'obstacle_front_deg', 'obstacle_front_half_deg',
                     'obstacle_confirm_m', 'emergency_stop_m', 'scan_timeout'):
            setattr(self, name, self.get_parameter(name).value)

        self.bio_anomaly = False
        self.obstacle_detected = False  # Hailo 원본 판정
        self.last_obstacle_time = None
        self.last_camera_time = None
        self.last_scan = None
        self.last_scan_time = None
        self.obstacle_reason = ''
        self.last_command = None

        self.last_raw = None
        self.last_raw_time = None
        self.raw_fresh = False

        self.create_subscription(Bool, '/sensors/bio_anomaly', self._on_bio_anomaly, 10)
        self.create_subscription(Bool, '/perception/obstacle_detected', self._on_obstacle_detected, 10)
        self.create_subscription(Twist, '/cmd_vel_raw', self._on_cmd_vel_raw, 10)
        # camera_info는 영상 프레임마다 같이 오는 작은 메시지라 카메라 심장박동으로 쓴다.
        self.create_subscription(CameraInfo, '/camera/camera_info', self._on_camera_info,
                                 qos_profile_sensor_data)
        # ydlidar는 SensorDataQoS(BEST_EFFORT)로 발행한다. 기본 QoS로 구독하면
        # 호환되지 않아 메시지가 하나도 안 들어온다.
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

        self.state_pub = self.create_publisher(String, '/control/driving_state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.1, self._decide_and_publish)  # 10Hz

    def _on_bio_anomaly(self, msg):
        if self.bio_latch and self.bio_anomaly:
            return  # 한 번 걸린 래치는 풀지 않는다
        if msg.data and not self.bio_anomaly and self.bio_latch:
            self.get_logger().warn('운전자 이상 래치 — 재시작 전까지 해제되지 않는다')
        self.bio_anomaly = msg.data

    def _on_obstacle_detected(self, msg):
        self.obstacle_detected = msg.data
        self.last_obstacle_time = self.get_clock().now()

    def _on_camera_info(self, _msg):
        self.last_camera_time = self.get_clock().now()

    def _on_scan(self, msg):
        self.last_scan = msg
        self.last_scan_time = self.get_clock().now()

    def _fresh(self, t, timeout):
        return t is not None and (self.get_clock().now() - t).nanoseconds * 1e-9 < timeout

    def _judge_obstacle(self):
        scan_fresh = self._fresh(self.last_scan_time, self.scan_timeout)
        front_min = None
        if scan_fresh:
            s = self.last_scan
            front_min = sector_min(s.ranges, s.angle_min, s.angle_increment,
                                   self.obstacle_front_deg, self.obstacle_front_half_deg,
                                   s.range_min, s.range_max)
        stop, reason = judge_obstacle(
            self.obstacle_detected,
            camera_fresh=self._fresh(self.last_camera_time, self.perception_timeout),
            hailo_fresh=self._fresh(self.last_obstacle_time, self.perception_timeout),
            scan_fresh=scan_fresh, front_min=front_min,
            confirm_m=self.obstacle_confirm_m, emergency_m=self.emergency_stop_m,
            require_perception=self.require_perception, fuse_lidar=self.fuse_lidar)
        if reason != self.obstacle_reason:
            # rclpy 로거는 "같은 호출 위치(파일:줄)"의 심각도가 호출마다 바뀌는 걸 허용하지
            # 않는다 — 한 줄에서 warn/info를 골라 쓰면 두 번째로 다른 쪽이 불리는 순간
            # ValueError('Logger severity cannot be changed between calls.')로 죽는다
            # (실측: 장애물 유무가 바뀌면서 실제로 발생, decision_maker_node가 죽어
            # /cmd_vel이 아예 안 나가고 차가 멈춰 있었다). 호출 지점을 분리해서 피한다.
            if reason:
                if stop:
                    self.get_logger().warn(f'장애물 판단: {reason}')
                else:
                    self.get_logger().info(f'장애물 판단: {reason}')
            self.obstacle_reason = reason
        return stop

    def _on_cmd_vel_raw(self, msg):
        self.last_raw = msg
        self.last_raw_time = self.get_clock().now()

    def _decide_and_publish(self):
        command = self.decision_maker.decide(self.bio_anomaly, self._judge_obstacle())

        state_msg = String()
        state_msg.data = command
        self.state_pub.publish(state_msg)

        if command != self.last_command:
            self.get_logger().info(f'주행 상태 변경: {command}')
            self.last_command = command

        if command == COMMAND_NORMAL:
            self.cmd_vel_pub.publish(self._gated_drive_cmd())
            
        elif "MRM" in command or self.bio_anomaly:
            # [변경됨] 시간 계산 로직(타이머) 삭제!
            # lane_follower_node가 알아서 우측 차선을 인식해 갓길로 이동하고 속도를 0으로 만들어 주므로,
            # 통제소는 그 명령을 차단하지 않고 그대로 바퀴로 보내주기만 하면 됩니다.
            self.cmd_vel_pub.publish(self._gated_drive_cmd())
            
        else:
            # 장애물 감지(EMERGENCY_BRAKE) 등 즉각적인 충돌 위험 시에는
            # 차선이고 뭐고 무시하고 즉시 급정거를 꽂아 넣습니다.
            self.cmd_vel_pub.publish(Twist())

    def _gated_drive_cmd(self):
        """NORMAL 및 MRM 상태에서 내보낼 주행 명령. 신선한 /cmd_vel_raw가 없으면 정지."""
        now = self.get_clock().now()
        fresh = (
            self.last_raw_time is not None
            and (now - self.last_raw_time).nanoseconds * 1e-9 < self.cmd_vel_timeout
        )
        if fresh != self.raw_fresh:
            if fresh:
                self.get_logger().info('/cmd_vel_raw 수신 시작 — 주행 명령 통과')
            else:
                self.get_logger().warn(
                    f'/cmd_vel_raw {self.cmd_vel_timeout:.1f}초 이상 끊김 — 정지 명령 발행')
            self.raw_fresh = fresh
        return self.last_raw if fresh else Twist()


def main(args=None):
    rclpy.init(args=args)
    node = DecisionMakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()