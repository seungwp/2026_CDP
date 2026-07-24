import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from safecar_perception.vision_detector import VisionDetector


class VisionDetectorNode(Node):
    """'/camera/image_raw'를 구독해 차선 오프셋을 계산·publish한다. (인지부)

    - '/perception/lane_offset' (Float32, -1~+1): 차선을 찾은 프레임에서만 publish.
      구독자(lane_follower)는 이 토픽의 신선도로 차선 유실을 판단한다.
    - '/perception/lane_image' (Image): 검출 선분/차로 중심이 그려진 디버그 영상 (튜닝용).
    - 장애물 인식은 이 노드가 아니라 Hailo NPU 노드가 '/perception/obstacle_detected'로 담당.

    카메라 자체는 이 노드가 열지 않는다 — camera_ros(camera_node)가 열어서
    '/camera/image_raw'로 publish하고, 이 노드는 구독만 한다.
    """

    def __init__(self):
        super().__init__('vision_detector_node')
        self.detector = VisionDetector()
        self.bridge = CvBridge()
        self.lane_visible = False

        self.offset_pub = self.create_publisher(Float32, '/perception/lane_offset', 10)
        self.debug_pub = self.create_publisher(Image, '/perception/lane_image', 10)
        self.create_subscription(Image, '/camera/image_raw', self._on_image, 10)

    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패: {e}')
            return

        debug_frame, offset = self.detector.process_frame(frame)

        if offset is not None:
            self.offset_pub.publish(Float32(data=offset))
        if (offset is not None) != self.lane_visible:
            self.lane_visible = offset is not None
            self.get_logger().info('차선 인식됨' if self.lane_visible else '차선 유실')

        debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
