import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class MockObstacleNode(Node):
    """
    Hailo AI 객체 인식 노드를 대체하는 임시 노드.
    현재는 주행 테스트를 위해 항상 '장애물 없음(False)'을 발행한다.
    """
    def __init__(self):
        super().__init__('mock_obstacle_node')
        self.publisher_ = self.create_publisher(Bool, '/perception/obstacle_detected', 10)
        # 10Hz로 False 신호 발행
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Mock Obstacle 노드가 시작되었습니다. (항상 안전 신호 발행)')

    def timer_callback(self):
        msg = Bool()
        msg.data = False
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()