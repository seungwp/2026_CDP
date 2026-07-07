import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

from safecar_msgs.command_protocol import (
    COMMAND_EMERGENCY_BRAKE,
    COMMAND_MRM_PULL_OVER,
)
from safecar_control.decision_maker import DecisionMaker


class DecisionMakerNode(Node):
    """비전/생체신호 토픽을 구독해 주행 상태를 판단하고, 개입이 필요할 때만 cmd_vel을 publish한다. (제어부)

    평상시(NORMAL)는 cmd_vel을 건드리지 않는다 — 정상 주행은 이 노드의 책임이 아니라
    별도의 주행/조향 담당 노드가 맡고, 이 노드는 안전 감독(fail-safe) 레이어로만 동작한다.
    """

    def __init__(self):
        super().__init__('decision_maker_node')
        self.decision_maker = DecisionMaker()

        self.bio_anomaly = False
        self.obstacle_detected = False
        self.last_command = None

        self.create_subscription(Bool, '/sensors/bio_anomaly', self._on_bio_anomaly, 10)
        self.create_subscription(Bool, '/perception/obstacle_detected', self._on_obstacle_detected, 10)

        self.state_pub = self.create_publisher(String, '/control/driving_state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.1, self._decide_and_publish)  # 10Hz

    def _on_bio_anomaly(self, msg):
        self.bio_anomaly = msg.data

    def _on_obstacle_detected(self, msg):
        self.obstacle_detected = msg.data

    def _decide_and_publish(self):
        command = self.decision_maker.decide(self.bio_anomaly, self.obstacle_detected)

        state_msg = String()
        state_msg.data = command
        self.state_pub.publish(state_msg)

        if command != self.last_command:
            self.get_logger().info(f'주행 상태 변경: {command}')
            self.last_command = command

        if command == COMMAND_EMERGENCY_BRAKE:
            self.cmd_vel_pub.publish(Twist())  # 급제동: 속도 0
        elif command == COMMAND_MRM_PULL_OVER:
            # TODO: 차선 기반 갓길 대피 경로/조향 로직 설계 필요. 지금은 정지만 수행.
            self.cmd_vel_pub.publish(Twist())


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
