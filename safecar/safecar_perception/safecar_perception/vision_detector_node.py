import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np


class VisionDetector:
    """카메라 프레임을 버드아이(top-down)로 변환해 차선 중심선을 다항식으로 피팅하고,
    차량 기준 정규화 중심선 점들과 횡방향 오프셋을 계산한다.

    좌표 규약 (제어부와 공유):
      - offset: 차량 위치(맨 아래 행)의 횡오차. +1 = 차로 중심이 오른쪽(→ 좌회전 필요),
                -1 = 왼쪽, 0 = 일치. clip(-1, +1).
      - path 점: (lateral, forward) 정규화 좌표. lateral +는 오른쪽, forward +는 전방.
                 (warp 폭의 절반)을 1.0으로 정규화 → offset 규약과 스케일 일치.
    """

    # --- 색 검출 파라미터 (기존 값 유지: 트랙 조명/차선색에 맞춰 여기만 손대면 됨) ---
    USE_WHITE = True

    def __init__(self, persp_src, num_windows=9, window_margin=60, min_pix=50,
                 num_path_points=6, lane_width_frac=1.0):
        # persp_src: np.float32 (4,2) — [top-left, top-right, bottom-right, bottom-left]
        self.persp_src = np.asarray(persp_src, dtype=np.float32).reshape(4, 2)
        self.num_windows = num_windows
        self.window_margin = window_margin
        self.min_pix = min_pix
        self.num_path_points = num_path_points

        # 워프 출력 크기는 첫 프레임에서 원본 크기로 확정한다.
        self.warp_w = None
        self.warp_h = None
        self.M = None
        self.Minv = None

        # 단일 차선만 보일 때 사용할 차로 폭(정규화). 양쪽이 보이면 EMA로 갱신.
        self.lane_width = lane_width_frac  # 정규화 단위(1.0 = warp 반폭)
        self._lane_width_px = None         # 픽셀 단위 내부 상태

    # ── 원근 변환 준비 ──────────────────────────────────────────────
    def _ensure_warp(self, w, h):
        if self.warp_w == w and self.warp_h == h and self.M is not None:
            return
        self.warp_w, self.warp_h = w, h
        dst = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
        self.M = cv2.getPerspectiveTransform(self.persp_src, dst)
        self.Minv = cv2.getPerspectiveTransform(dst, self.persp_src)

    # ── 색 마스크 (기존 로직 재사용) ───────────────────────────────
    def _color_mask(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([18, 70, 70]), np.array([40, 255, 255]))  # 노랑 계열
        if self.USE_WHITE:
            mask_white = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 25, 255]))
            mask = cv2.bitwise_or(mask, mask_white)
        return mask

    # ── sliding window로 좌/우 차선 픽셀 수집 ──────────────────────
    def _sliding_window(self, warped):
        h, w = warped.shape
        histogram = np.sum(warped[h // 2:, :], axis=0)
        midpoint = w // 2
        leftx_base = int(np.argmax(histogram[:midpoint]))
        rightx_base = int(np.argmax(histogram[midpoint:]) + midpoint)

        window_height = h // self.num_windows
        nonzero = warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx_current, rightx_current = leftx_base, rightx_base
        margin, minpix = self.window_margin, self.min_pix
        left_inds, right_inds = [], []

        for win in range(self.num_windows):
            y_lo = h - (win + 1) * window_height
            y_hi = h - win * window_height
            for base, store in ((leftx_current, left_inds), (rightx_current, right_inds)):
                good = ((nonzeroy >= y_lo) & (nonzeroy < y_hi) &
                        (nonzerox >= base - margin) & (nonzerox < base + margin)).nonzero()[0]
                store.append(good)

            lg = left_inds[-1]
            rg = right_inds[-1]
            if len(lg) > minpix:
                leftx_current = int(np.mean(nonzerox[lg]))
            if len(rg) > minpix:
                rightx_current = int(np.mean(nonzerox[rg]))

        left_inds = np.concatenate(left_inds) if left_inds else np.array([], dtype=int)
        right_inds = np.concatenate(right_inds) if right_inds else np.array([], dtype=int)

        lx, ly = nonzerox[left_inds], nonzeroy[left_inds]
        rx, ry = nonzerox[right_inds], nonzeroy[right_inds]
        return lx, ly, rx, ry

    # ── 메인 처리 ──────────────────────────────────────────────────
    def process_frame(self, frame):
        h, w = frame.shape[:2]
        self._ensure_warp(w, h)
        half = w / 2.0  # 정규화 기준(warp 반폭)

        mask = self._color_mask(frame)
        warped = cv2.warpPerspective(mask, self.M, (w, h), flags=cv2.INTER_NEAREST)

        lx, ly, rx, ry = self._sliding_window(warped)

        min_fit = 200  # 피팅에 필요한 최소 픽셀 수
        left_ok = len(lx) >= min_fit
        right_ok = len(rx) >= min_fit

        left_fit = np.polyfit(ly, lx, 2) if left_ok else None
        right_fit = np.polyfit(ry, rx, 2) if right_ok else None

        def poly_x(fit, y):
            return fit[0] * y * y + fit[1] * y + fit[2]

        # 중심선 계산 (양쪽/한쪽/없음)
        if left_fit is not None and right_fit is not None:
            width_px = poly_x(right_fit, h - 1) - poly_x(left_fit, h - 1)
            if width_px > 20:  # 유효 차로폭이면 기억 갱신(EMA)
                self._lane_width_px = (width_px if self._lane_width_px is None
                                       else 0.8 * self._lane_width_px + 0.2 * width_px)

            def center_x(y):
                return 0.5 * (poly_x(left_fit, y) + poly_x(right_fit, y))
        elif left_fit is not None or right_fit is not None:
            fit = left_fit if left_fit is not None else right_fit
            sign = +1.0 if left_fit is not None else -1.0  # 왼쪽만 보이면 +half폭, 오른쪽만이면 -
            offw = (self._lane_width_px / 2.0) if self._lane_width_px else (self.lane_width * half)

            def center_x(y):
                return poly_x(fit, y) + sign * offw
        else:
            debug = self._draw(frame, None, None, None, None)
            return debug, None, None

        # 맨 아래 행 오프셋 (기존 규약 유지)
        offset = float(np.clip((center_x(h - 1) - half) / half, -1.0, 1.0))

        # 정규화 중심선 lookahead 점들: forward 오름차순 (차량=맨 아래)
        ys = np.linspace(h - 1, int(h * 0.35), self.num_path_points)
        path = []
        for y in ys:
            lat = (center_x(y) - half) / half        # +오른쪽
            fwd = (h - 1 - y) / half                 # +전방
            path.append((float(lat), float(fwd)))

        debug = self._draw(frame, left_fit, right_fit, center_x, offset)
        return debug, offset, path

    # ── 디버그 오버레이 (원본에 trapezoid + 중심선 역투영) ─────────
    def _draw(self, frame, left_fit, right_fit, center_x, offset):
        debug = frame.copy()
        h, w = frame.shape[:2]

        # 원근 변환 영역(trapezoid) — persp_src 캘리브레이션 확인용
        cv2.polylines(debug, [self.persp_src.astype(np.int32)], True, (255, 0, 0), 2)

        def poly_x(fit, y):
            return fit[0] * y * y + fit[1] * y + fit[2]

        ys = np.linspace(h - 1, int(h * 0.35), 20)
        for fit, color in ((left_fit, (0, 255, 0)), (right_fit, (0, 255, 0))):
            if fit is None:
                continue
            pts = np.array([[poly_x(fit, y), y] for y in ys], dtype=np.float32).reshape(-1, 1, 2)
            pts = cv2.perspectiveTransform(pts, self.Minv).astype(np.int32)
            cv2.polylines(debug, [pts], False, color, 2)

        if center_x is not None:
            cpts = np.array([[center_x(y), y] for y in ys], dtype=np.float32).reshape(-1, 1, 2)
            cpts = cv2.perspectiveTransform(cpts, self.Minv).astype(np.int32)
            cv2.polylines(debug, [cpts], False, (0, 255, 255), 2)

        cv2.line(debug, (w // 2, int(h * 0.5)), (w // 2, h), (128, 128, 128), 1)
        if offset is not None:
            cv2.putText(debug, f"offset={offset:+.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        else:
            cv2.putText(debug, "NO LANE", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return debug


class VisionDetectorNode(Node):
    """실제 ROS2 환경과 VisionDetector 로직을 연결해 주는 래퍼 노드."""

    def __init__(self):
        super().__init__('vision_detector_node')

        # 원근 변환 4점(top-left, top-right, bottom-right, bottom-left), 640x480 기준 기본값.
        # 트랙에서 /perception/lane_image를 보며 이 값을 조정(캘리브레이션)한다.
        self.declare_parameter('persp_src',
                               [200.0, 280.0, 440.0, 280.0, 600.0, 470.0, 40.0, 470.0])
        self.declare_parameter('num_windows', 9)
        self.declare_parameter('window_margin', 60)
        self.declare_parameter('min_pix', 50)
        self.declare_parameter('num_path_points', 6)
        self.declare_parameter('lane_width_frac', 1.0)  # 단일차선 시 반대편 추정 폭(정규화)

        src = self.get_parameter('persp_src').value
        self.detector = VisionDetector(
            persp_src=src,
            num_windows=self.get_parameter('num_windows').value,
            window_margin=self.get_parameter('window_margin').value,
            min_pix=self.get_parameter('min_pix').value,
            num_path_points=self.get_parameter('num_path_points').value,
            lane_width_frac=self.get_parameter('lane_width_frac').value,
        )
        self.bridge = CvBridge()
        self.get_logger().info("[System] Vision: 버드아이 차선 인식 노드 초기화 완료.")

        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.offset_pub = self.create_publisher(Float32, '/perception/lane_offset', 10)
        self.path_pub = self.create_publisher(Float32MultiArray, '/perception/lane_path', 10)
        self.image_pub = self.create_publisher(Image, '/perception/lane_image', 10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            debug_frame, offset, path = self.detector.process_frame(cv_image)

            if offset is not None:
                self.offset_pub.publish(Float32(data=offset))
            if path:
                # [lat0, fwd0, lat1, fwd1, ...] (정규화, forward 오름차순)
                flat = [v for pt in path for v in pt]
                self.path_pub.publish(Float32MultiArray(data=flat))

            # 캘리브레이션 중엔 항상 발행(원격 rqt 구독자 수 감지가 불안정할 수 있어 게이트 제거)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, 'bgr8')
            debug_msg.header = msg.header
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
