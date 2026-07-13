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
    - 비상(EMERGENCY_BRAKE): 주행 명령을 차단하고 정지로 오버라이드.
    - MRM_PULL_OVER: lane_follower가 자체적으로 갓길 주행 후 정차하므로 명령을 통과시킨다.
    """

    def __init__(self):
        super().__init__('decision_maker_node')
        self.decision_maker = DecisionMaker()

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