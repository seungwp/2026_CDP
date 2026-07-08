import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

from safecar_msgs.command_protocol import COMMAND_NORMAL
from safecar_control.decision_maker import DecisionMaker


class DecisionMakerNode(Node):
    """비전/생체신호 토픽을 구독해 주행 상태를 판단하고, /cmd_vel의 단일 게이트로 동작한다. (제어부)

    안전 게이트 구조: 주행 명령(teleop, 추후 차선 추종 노드)은 '/cmd_vel_raw'로 들어오고,
    '/cmd_vel'은 이 노드만 publish한다 — 비상 시 정지 명령이 주행 명령과 경쟁하지 않는다.
    - NORMAL: 신선한(cmd_vel_timeout 이내) /cmd_vel_raw를 10Hz로 통과시킨다.
    - 비상(EMERGENCY_BRAKE / MRM_PULL_OVER): 주행 명령을 차단하고 정지로 오버라이드.
    - /cmd_vel_raw가 timeout 동안 끊기면(Wi-Fi 단절, teleop 종료 등) 정지 —
      stella_md에는 자체 타임아웃이 없어 마지막 속도로 계속 달리는 문제를 여기서 막는다.
    """

    def __init__(self):
        super().__init__('decision_maker_node')
        self.decision_maker = DecisionMaker()

        # /cmd_vel_raw가 이 시간(초) 이상 끊기면 정지. 키를 눌렀을 때만 publish하는
        # teleop을 쓰면 키를 뗀 뒤 이 시간만큼 마지막 속도가 유지되다 멈춘다(데드맨 동작).
        self.declare_parameter('cmd_vel_timeout', 1.0)
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        self.bio_anomaly = False
        self.obstacle_detected = False
        self.last_command = None

        self.last_raw = None
        self.last_raw_time = None
        self.raw_fresh = False

        self.create_subscription(Bool, '/sensors/bio_anomaly', self._on_bio_anomaly, 10)
        self.create_subscription(Bool, '/perception/obstacle_detected', self._on_obstacle_detected, 10)
        self.create_subscription(Twist, '/cmd_vel_raw', self._on_cmd_vel_raw, 10)

        self.state_pub = self.create_publisher(String, '/control/driving_state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.1, self._decide_and_publish)  # 10Hz

    def _on_bio_anomaly(self, msg):
        self.bio_anomaly = msg.data

    def _on_obstacle_detected(self, msg):
        self.obstacle_detected = msg.data

    def _on_cmd_vel_raw(self, msg):
        self.last_raw = msg
        self.last_raw_time = self.get_clock().now()

    def _decide_and_publish(self):
        command = self.decision_maker.decide(self.bio_anomaly, self.obstacle_detected)

        state_msg = String()
        state_msg.data = command
        self.state_pub.publish(state_msg)

        if command != self.last_command:
            self.get_logger().info(f'주행 상태 변경: {command}')
            self.last_command = command

        if command == COMMAND_NORMAL:
            self.cmd_vel_pub.publish(self._gated_drive_cmd())
        else:
            # 비상: 주행 명령 차단, 정지 오버라이드.
            # TODO: MRM_PULL_OVER는 차선/스캔 기반 갓길 조향으로 교체 예정. 지금은 정지만 수행.
            self.cmd_vel_pub.publish(Twist())

    def _gated_drive_cmd(self):
        """NORMAL 상태에서 내보낼 주행 명령. 신선한 /cmd_vel_raw가 없으면 정지."""
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
