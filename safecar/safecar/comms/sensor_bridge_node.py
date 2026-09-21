import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class SensorBridgeNode(Node):
    """ESP32/STM32 등 외부 보드와 시리얼로 통신해 생체신호 등을 ROS2 토픽으로 중계한다. (통신부)

    TODO: ESP32/STM32의 실제 시리얼 프로토콜이 정해지면 pyserial로 읽어 파싱하도록 교체.
    지금은 예전 main.py에 있던 임시 시뮬레이션(anomaly_delay_sec 경과 시 이상 발생)을 유지.
    anomaly_delay_sec을 0 이하로 주면 이상 신호를 발생시키지 않는다 —
    teleop/주행 게이트 검증처럼 NORMAL 상태를 계속 유지해야 하는 테스트용.
    참고: 모터 제어는 이 노드가 아니라 STELLA N1의 기존 stella_md 노드가
    '/cmd_vel' 토픽으로 직접 담당한다 (시리얼 포트를 stella_md가 이미 점유함).
    """

    def __init__(self):
        super().__init__('sensor_bridge_node')
        self.declare_parameter('anomaly_delay_sec', 10.0)
        self.anomaly_delay_sec = self.get_parameter('anomaly_delay_sec').value

        self.bio_anomaly_pub = self.create_publisher(Bool, '/sensors/bio_anomaly', 10)
        self.start_time = time.time()
        self.create_timer(0.5, self._on_timer)

    def _on_timer(self):
        msg = Bool()
        msg.data = (
            self.anomaly_delay_sec > 0
            and (time.time() - self.start_time) > self.anomaly_delay_sec
        )
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
