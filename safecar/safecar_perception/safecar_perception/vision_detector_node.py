import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from safecar_perception.vision_detector import VisionDetector


class VisionDetectorNode(Node):
    """'/camera/image_raw'를 구독해 매 프레임 차선/장애물을 인식하고 결과를 publish한다. (인지부)

    카메라 자체는 이 노드가 열지 않는다 — 라즈베리파이 카메라 모듈(CSI)은
    camera_ros 패키지(camera_node)가 열어서 '/camera/image_raw'로 publish하고,
    이 노드는 그 이미지를 구독만 한다.
    """

    def __init__(self):
        super().__init__('vision_detector_node')

        hef_path = os.path.join(
            get_package_share_directory('safecar_perception'), 'models', 'yolov8n.hef'
        )
        self.detector = VisionDetector(hef_path=hef_path)
        self.bridge = CvBridge()

        self.obstacle_pub = self.create_publisher(Bool, '/perception/obstacle_detected', 10)
        self.create_subscription(Image, '/camera/image_raw', self._on_image, 10)

    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패: {e}')
            return

        _, obstacle_detected = self.detector.process_frame(frame)

        obstacle_msg = Bool()
        obstacle_msg.data = bool(obstacle_detected)
        self.obstacle_pub.publish(obstacle_msg)


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
