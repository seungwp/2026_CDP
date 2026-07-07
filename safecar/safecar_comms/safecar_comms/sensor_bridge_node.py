import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class SensorBridgeNode(Node):
    """ESP32/STM32 등 외부 보드와 시리얼로 통신해 생체신호 등을 ROS2 토픽으로 중계한다. (통신부)

    TODO: ESP32/STM32의 실제 시리얼 프로토콜이 정해지면 pyserial로 읽어 파싱하도록 교체.
    지금은 예전 main.py에 있던 임시 시뮬레이션(10초 경과 시 이상 발생)을 그대로 유지.
    참고: 모터 제어는 이 노드가 아니라 STELLA N1의 기존 stella_md 노드가
    '/cmd_vel' 토픽으로 직접 담당한다 (시리얼 포트를 stella_md가 이미 점유함).
    """

    def __init__(self):
        super().__init__('sensor_bridge_node')
        self.bio_anomaly_pub = self.create_publisher(Bool, '/sensors/bio_anomaly', 10)
        self.start_time = time.time()
        self.create_timer(0.5, self._on_timer)

    def _on_timer(self):
        msg = Bool()
        msg.data = (time.time() - self.start_time) > 10.0
        self.bio_anomaly_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
