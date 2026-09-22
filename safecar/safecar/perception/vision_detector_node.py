import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

from safecar.perception.vision_detector import VisionDetector
from safecar.control.scan_sectors import sector_min


class VisionDetectorNode(Node):
    """'/camera/image_raw'를 구독해 차선 오프셋을 계산·publish한다. (인지부)

    - '/perception/lane_offset' (Float32, -1~+1): 차선을 찾은 프레임에서만 publish.
      구독자(lane_follower)는 이 토픽의 신선도로 차선 유실을 판단한다.
    - '/perception/lane_heading' (Float32): 차선이 멀어지며 기우는 정도(+는 우측으로 휨).
      횡오차만 쓰면 곡선에서 밀리므로, 제어부가 이 값을 곡선 선제 조향에 쓴다.
    - '/perception/lane_image' (Image): 검출 선분/차로 중심이 그려진 디버그 영상 (튜닝용).
    - 장애물 인식은 이 노드가 아니라 Hailo NPU 노드가 '/perception/obstacle_detected'로 담당.

    '/control/mrm_mode'가 '우차로정차'가 되면 흰 차선 대신 갓길(노란 테이프)을 보도록
    전환한다(lane_follower가 결정해서 publish — decision_maker의 '/control/driving_state'만
    보면 자차로정차/우차로정차를 구분할 수 없어서 따로 받는다). 같은 '/perception/lane_offset'
    토픽을 계속 쓴다 — MRM 중엔 그 값이 '갓길선까지의 오프셋'이라는 뜻이 된다.

    카메라 자체는 이 노드가 열지 않는다 — camera_ros(camera_node)가 열어서
    '/camera/image_raw'로 publish하고, 이 노드는 구독만 한다.

    디버그 영상에는 라이다 전방/후방/우측 섹터 최소거리도 같이 찍는다(튜닝·시연용).
    섹터 중심각 기본값은 decision_maker(전방)·lane_follower(후방/우측)와 맞춰뒀다 —
    실제 판단은 각자 노드가 따로 하고, 여긴 화면에 보여주기만 한다.
    """

    def __init__(self):
        super().__init__('vision_detector_node')
        self.detector = VisionDetector()
        self.bridge = CvBridge()
        self.lane_visible = False
        self.shoulder_mode = False
        self.last_scan = None
        self.last_scan_time = None

        self.declare_parameter('scan_front_deg', 0.0)
        self.declare_parameter('scan_front_half_deg', 20.0)
        self.declare_parameter('scan_rear_deg', 180.0)
        self.declare_parameter('scan_side_deg', -90.0)
        self.declare_parameter('scan_sector_half_deg', 30.0)
        self.declare_parameter('scan_timeout', 1.0)
        for name in ('scan_front_deg', 'scan_front_half_deg', 'scan_rear_deg',
                     'scan_side_deg', 'scan_sector_half_deg', 'scan_timeout'):
            setattr(self, name, self.get_parameter(name).value)

        self.offset_pub = self.create_publisher(Float32, '/perception/lane_offset', 10)
        self.heading_pub = self.create_publisher(Float32, '/perception/lane_heading', 10)
        self.debug_pub = self.create_publisher(Image, '/perception/lane_image', 10)
        self.create_subscription(Image, '/camera/image_raw', self._on_image, 10)
        self.create_subscription(String, '/control/mrm_mode', self._on_mrm_mode, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

    def _on_mrm_mode(self, msg):
        active = (msg.data == '우차로정차')
        if active != self.shoulder_mode:
            self.shoulder_mode = active
            self.detector.set_shoulder_mode(active)
            self.get_logger().warn(
                '갓길(노란선) 추적 시작' if active else '흰 차선 추적으로 복귀')

    def _on_scan(self, msg):
        self.last_scan = msg
        self.last_scan_time = self.get_clock().now()

    def _draw_scan_overlay(self, frame):
        fresh = (
            self.last_scan is not None and self.last_scan_time is not None
            and (self.get_clock().now() - self.last_scan_time).nanoseconds * 1e-9
            < self.scan_timeout
        )
        if not fresh:
            cv2.putText(frame, '라이다: 없음', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
            return

        s = self.last_scan

        def fmt(label, center_deg, half_deg):
            d = sector_min(s.ranges, s.angle_min, s.angle_increment,
                           center_deg, half_deg, s.range_min, s.range_max)
            return f'{label} {d:.2f}m' if d is not None else f'{label} 없음'

        lines = [
            fmt('전방', self.scan_front_deg, self.scan_front_half_deg),
            fmt('후방', self.scan_rear_deg, self.scan_sector_half_deg),
            fmt('우측', self.scan_side_deg, self.scan_sector_half_deg),
        ]
        for i, text in enumerate(lines):
            cv2.putText(frame, text, (10, 60 + i * 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패: {e}')
            return

        debug_frame, offset, heading = self.detector.process_frame(frame)
        self._draw_scan_overlay(debug_frame)

        if offset is not None:
            self.offset_pub.publish(Float32(data=offset))
            self.heading_pub.publish(Float32(data=heading))
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
