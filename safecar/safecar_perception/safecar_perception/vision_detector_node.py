import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionDetector:
    """카메라 프레임에서 차선을 인식해 차로 중심 대비 횡방향 오프셋을 계산한다."""
    ROI_TOP = 0.45        
    Y_EVAL = 0.7          
    MIN_ABS_SLOPE = 0.3   
    HALF_LANE_PX = 340    
    USE_WHITE = False     

    def __init__(self):
        pass # Node 클래스에서 초기화 로그를 띄우므로 여기선 생략

    def process_frame(self, frame):
        height, width = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_yellow = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([30, 255, 255]))
        mask = cv2.inRange(hsv, np.array([18, 70, 70]), np.array([40, 255, 255]))
        
        if self.USE_WHITE:
            mask_white = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 25, 255]))
            mask = cv2.bitwise_or(mask, mask_white)

        edges = cv2.Canny(mask, 50, 150)
        roi_top = int(height * self.ROI_TOP)
        roi = np.zeros_like(edges)
        cv2.rectangle(roi, (0, roi_top), (width, height), 255, -1)
        masked_edges = cv2.bitwise_and(edges, roi)

        lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 50,
                                minLineLength=40, maxLineGap=120)

        debug = frame.copy()
        cv2.line(debug, (width // 2, roi_top), (width // 2, height), (255, 0, 0), 1)

        y_eval = int(height * self.Y_EVAL)
        left_xs, right_xs = [], []
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                dx, dy = x2 - x1, y2 - y1
                if dx == 0:
                    x_at = float(x1)
                else:
                    slope = dy / dx
                    if abs(slope) < self.MIN_ABS_SLOPE:
                        continue
                    x_at = x1 + (y_eval - y1) / slope
                
                if not (-width <= x_at < 2 * width):
                    continue
                    
                cv2.line(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
                if x_at < width / 2:
                    left_xs.append(x_at)
                else:
                    right_xs.append(x_at)

        if left_xs and right_xs:
            center = (np.median(left_xs) + np.median(right_xs)) / 2.0
        elif left_xs:
            center = np.median(left_xs) + self.HALF_LANE_PX
        elif right_xs:
            center = np.median(right_xs) - self.HALF_LANE_PX
        else:
            cv2.putText(debug, "NO LANE", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return debug, None

        offset = float(np.clip((center - width / 2) / (width / 2), -1.0, 1.0))

        cv2.circle(debug, (int(center), y_eval), 6, (0, 0, 255), -1)
        cv2.putText(debug, f"offset={offset:+.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return debug, offset


class VisionDetectorNode(Node):
    """실제 ROS2 환경과 VisionDetector 로직을 연결해 주는 래퍼 노드"""
    
    def __init__(self):
        super().__init__('vision_detector_node')
        self.get_logger().info("[System] Vision: OpenCV 차선 인식 노드 초기화 완료.")
        
        self.detector = VisionDetector()
        self.bridge = CvBridge()
        
        # 카메라 영상 구독 (입력)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # 제어부로 보낼 오프셋 및 디버그 영상 퍼블리셔 (출력)
        self.offset_pub = self.create_publisher(Float32, '/perception/lane_offset', 10)
        self.image_pub = self.create_publisher(Image, '/perception/lane_image', 10)

    def image_callback(self, msg):
        try:
            # 1. ROS Image 메시지를 OpenCV 포맷으로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 2. 핵심 인식 로직 수행
            debug_frame, offset = self.detector.process_frame(cv_image)
            
            # 3. 오프셋이 찾아지면 제어부로 퍼블리시
            if offset is not None:
                offset_msg = Float32()
                offset_msg.data = offset
                self.offset_pub.publish(offset_msg)
                
            # 4. 디버그/대시보드용 이미지 퍼블리시
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, 'bgr8')
            self.image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f"이미지 처리 중 오류 발생: {e}")


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