"""
UFLD (Ultra-Fast Lane Detection) Hailo-8 NPU 차선 감지 노드

RPi5 + Hailo-8 실물 하드웨어 전용.
시뮬레이션에서는 vision_detector_node.py 를 사용할 것.

HEF 모델 다운로드:
    mkdir -p ~/ros2_ws/src/safecar/safecar_perception/models
    wget -O ~/ros2_ws/src/safecar/safecar_perception/models/ufld_culane.hef \\
      https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/LaneDetection/ufld/\\
ultra_fast_lane_detection_culane/2022-05-10/ultra_fast_lane_detection_culane.hef

구독: /camera/image_raw   (sensor_msgs/Image)
발행: /perception/lane_offset (std_msgs/Float32, -1.0 ~ +1.0)
      /perception/lane_image  (sensor_msgs/Image, 디버그용)

오프셋 규약 (vision_detector_node 와 동일):
  +1.0 = 차로 중심이 화면 오른쪽 → 좌회전 필요
  -1.0 = 차로 중심이 화면 왼쪽  → 우회전 필요
   0.0 = 차로 중심과 일치
"""

import os
import threading
import queue

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

# Hailo SDK — RPi5 에서만 설치됨. x86 개발 환경에서는 ImportError 허용.
try:
    from hailo_platform import (
        HEF, VDevice, HailoSchedulingAlgorithm,
        InputVStreams, OutputVStreams,
        FormatType,
    )
    HAILO_AVAILABLE = True
except ImportError:
    HAILO_AVAILABLE = False


# ── UFLD CULane 모델 설정 ────────────────────────────────────────────
# CULane 학습 기준: 800×288 입력, 4차선, 행 앵커 18개, 열 클래스 100+1개
UFLD_INPUT_W  = 800
UFLD_INPUT_H  = 288
NUM_LANES     = 4
NUM_ROW_ANCHORS = 18
NUM_COL_CLASSES = 101   # 100 위치 + 1 ("차선 없음" 클래스)

# 이미지 세로 42%~100% 구간에 행 앵커 18개 배치 (CULane 논문 기준)
ROW_ANCHORS = np.linspace(0.42, 1.0, NUM_ROW_ANCHORS)

# 한쪽 차선만 보일 때 가정하는 차로 반폭 (원본 이미지 픽셀)
HALF_LANE_PX = 160

DEFAULT_HEF = os.path.join(
    os.path.dirname(__file__), '..', '..', '..', '..', 'share',
    'safecar_perception', 'models', 'ufld_culane.hef'
)


# ── 순수 후처리 로직 ─────────────────────────────────────────────────

def ufld_postprocess(raw_output: np.ndarray, orig_w: int, orig_h: int):
    """
    UFLD NPU 출력 → 차선 픽셀 좌표 목록

    raw_output : 1D 또는 3D float32 배열.
                 shape = (NUM_LANES, NUM_ROW_ANCHORS, NUM_COL_CLASSES)
                 또는 그 flattened 버전.
    반환       : List[List[(x, y)]]  각 차선의 (원본 이미지 픽셀) 점 목록
    """
    pred = raw_output.reshape(NUM_LANES, NUM_ROW_ANCHORS, NUM_COL_CLASSES)
    col_step = UFLD_INPUT_W / (NUM_COL_CLASSES - 1)

    lanes = []
    for lane_i in range(NUM_LANES):
        points = []
        for row_i in range(NUM_ROW_ANCHORS):
            logits = pred[lane_i, row_i]            # [101]
            best_col = int(np.argmax(logits[:-1]))  # 마지막 = "없음"
            if logits[best_col] > logits[-1]:        # 차선 있음
                x = int(best_col * col_step * orig_w / UFLD_INPUT_W)
                y = int(ROW_ANCHORS[row_i] * orig_h)
                points.append((x, y))
        if len(points) >= 3:
            lanes.append(points)

    return lanes


def lanes_to_offset(lanes, img_w: int):
    """
    감지된 차선 → 정규화 오프셋 (-1 ~ +1)
    중앙에서 가장 가까운 좌/우 차선을 기준으로 차로 중심 추정.
    """
    if not lanes:
        return None

    cx = img_w / 2.0

    def bottom_x(lane):
        return lane[-1][0]

    left_lanes  = [l for l in lanes if bottom_x(l) < cx]
    right_lanes = [l for l in lanes if bottom_x(l) >= cx]

    if left_lanes and right_lanes:
        lx = bottom_x(max(left_lanes,  key=bottom_x))
        rx = bottom_x(min(right_lanes, key=bottom_x))
        center = (lx + rx) / 2.0
    elif left_lanes:
        lx = bottom_x(max(left_lanes, key=bottom_x))
        center = lx + HALF_LANE_PX
    else:
        rx = bottom_x(min(right_lanes, key=bottom_x))
        center = rx - HALF_LANE_PX

    return float(np.clip((center - cx) / cx, -1.0, 1.0))


def draw_lanes(frame, lanes, offset):
    """디버그 이미지: 감지된 차선과 오프셋 표시"""
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255)]
    for i, lane in enumerate(lanes):
        color = colors[i % len(colors)]
        for j in range(len(lane) - 1):
            cv2.line(frame, lane[j], lane[j + 1], color, 3)
        for pt in lane:
            cv2.circle(frame, pt, 4, color, -1)

    h, w = frame.shape[:2]
    cx = w // 2
    if offset is not None:
        est_cx = int(cx + offset * cx)
        cv2.line(frame, (est_cx, 0), (est_cx, h), (0, 255, 255), 2)
        cv2.putText(frame, f'offset={offset:+.2f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    else:
        cv2.putText(frame, 'NO LANE', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    cv2.line(frame, (cx, 0), (cx, h), (128, 128, 128), 1)
    return frame


# ── Hailo 추론 래퍼 ──────────────────────────────────────────────────

class HailoUFLD:
    """Hailo Platform Python API 로 UFLD HEF 를 동기 실행하는 래퍼."""

    def __init__(self, hef_path: str):
        if not HAILO_AVAILABLE:
            raise RuntimeError('hailo_platform 패키지가 설치되지 않았습니다. '
                               'RPi5 + Hailo SDK 환경에서 실행하세요.')
        self._hef  = HEF(hef_path)
        self._vdev = VDevice()
        self._ng   = self._vdev.configure(self._hef)[0]

        self._in_info  = self._hef.get_input_vstream_infos()[0]
        self._out_info = self._hef.get_output_vstream_infos()[0]

        self._in_params  = {self._in_info.name:
                            self._ng.make_input_vstream_params(
                                quantized=False, format_type=FormatType.FLOAT32)}
        self._out_params = {self._out_info.name:
                            self._ng.make_output_vstream_params(
                                quantized=False, format_type=FormatType.FLOAT32)}

        self._lock = threading.Lock()
        self._ng.activate(None).__enter__()

    def infer(self, img_bgr: np.ndarray) -> np.ndarray:
        """
        img_bgr : 임의 크기 BGR 이미지 (HWC uint8)
        반환    : raw output float32 array
        """
        # 전처리: 리사이즈 → RGB 정규화
        inp = cv2.resize(img_bgr, (UFLD_INPUT_W, UFLD_INPUT_H))
        inp = cv2.cvtColor(inp, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        inp = inp[np.newaxis, ...]   # [1, H, W, 3]

        with self._lock:
            with InputVStreams(self._ng, self._in_params) as ivs:
                with OutputVStreams(self._ng, self._out_params) as ovs:
                    ivs[self._in_info.name].send(inp)
                    raw = ovs[self._out_info.name].recv()
        return raw


# ── ROS2 노드 ────────────────────────────────────────────────────────

class UFLDHailoNode(Node):

    def __init__(self):
        super().__init__('ufld_hailo_node')

        self.declare_parameter('hef_path', DEFAULT_HEF)
        self.declare_parameter('offset_smoothing', 0.6)

        hef_path = self.get_parameter('hef_path').value
        self._smoothing = self.get_parameter('offset_smoothing').value

        if not HAILO_AVAILABLE:
            self.get_logger().error(
                'hailo_platform 미설치 — RPi5 + Hailo SDK 환경에서 실행하세요.')
            raise SystemExit(1)

        if not os.path.isfile(hef_path):
            self.get_logger().error(
                f'HEF 파일 없음: {hef_path}\n'
                'README 의 wget 명령으로 다운로드하세요.')
            raise SystemExit(1)

        self._hailo = HailoUFLD(hef_path)
        self._bridge = CvBridge()

        self._offset_pub = self.create_publisher(Float32, '/perception/lane_offset', 10)
        self._debug_pub  = self.create_publisher(Image,   '/perception/lane_image',  10)

        # 추론은 별도 스레드에서 처리 (GIL 우회 + ROS 콜백 블로킹 방지)
        self._q = queue.Queue(maxsize=2)
        self._infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
        self._infer_thread.start()

        self._last_offset = 0.0

        self.create_subscription(Image, '/camera/image_raw', self._on_image, 10)
        self.get_logger().info(f'UFLD Hailo 노드 시작  hef={hef_path}')

    # ── 이미지 수신 ────────────────────────────────────────────────

    def _on_image(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge 오류: {e}')
            return

        # 큐가 가득 차면 가장 오래된 프레임 버리고 최신 프레임 넣기
        if self._q.full():
            try:
                self._q.get_nowait()
            except queue.Empty:
                pass
        try:
            self._q.put_nowait((frame, msg.header))
        except queue.Full:
            pass

    # ── 추론 스레드 ────────────────────────────────────────────────

    def _infer_loop(self):
        while rclpy.ok():
            try:
                frame, header = self._q.get(timeout=1.0)
            except queue.Empty:
                continue

            try:
                raw = self._hailo.infer(frame)
            except Exception as e:
                self.get_logger().error(f'Hailo 추론 오류: {e}')
                continue

            h, w = frame.shape[:2]
            lanes  = ufld_postprocess(raw, w, h)
            offset = lanes_to_offset(lanes, w)

            # EMA 스무딩
            if offset is not None:
                self._last_offset = (self._smoothing * self._last_offset
                                     + (1 - self._smoothing) * offset)
                self._offset_pub.publish(Float32(data=self._last_offset))

            # 디버그 이미지 (구독자 있을 때만)
            if self._debug_pub.get_subscription_count() > 0:
                debug = draw_lanes(frame.copy(), lanes,
                                   self._last_offset if offset is not None else None)
                img_msg = self._bridge.cv2_to_imgmsg(debug, 'bgr8')
                img_msg.header = header
                self._debug_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UFLDHailoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
