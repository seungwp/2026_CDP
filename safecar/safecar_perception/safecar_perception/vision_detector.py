import cv2
import numpy as np
import time
import os

# HailoRT 라이브러리 (라즈베리파이 실차 환경에서만 동작)
try:
    from hailo_platform import VDevice, HailoStreamInterface, InferVStream, ConfigureParams
    HAILO_AVAILABLE = True
except ImportError:
    print("[Warning] HailoRT 라이브러리가 없습니다. NPU 추론은 건너뜁니다.")
    HAILO_AVAILABLE = False


class VisionDetector:
    """카메라 프레임을 받아 차선(OpenCV)과 전방 장애물(Hailo-8 NPU)을 인식한다. (인지부, ROS 비의존 순수 로직)"""

    def __init__(self, hef_path):
        self.hef_path = hef_path
        self.npu_ready = False

        if HAILO_AVAILABLE and os.path.exists(self.hef_path):
            self._init_hailo()
        else:
            print("[System] Vision: NPU를 사용할 수 없어 OpenCV 차선 인식만 수행합니다.")

    def _init_hailo(self):
        """Hailo-8 디바이스 및 모델 초기화"""
        try:
            self.target = VDevice()
            self.hef = HEF(self.hef_path)
            configure_params = ConfigureParams.create_from_hef(self.hef, interface=HailoStreamInterface.PCIe)
            self.network_group = self.target.configure(self.hef, configure_params)[0]
            self.network_group_params = self.network_group.create_params()
            print("[System] Hailo-8 NPU 모델 로드 완료.")
            self.npu_ready = True
        except Exception as e:
            print(f"[Hailo Error] NPU 초기화 실패: {e}")

    def process_frame(self, frame):
        result_frame = frame.copy()
        obstacle_detected = False

        # --- 1. 차선 및 갓길 검출 (OpenCV) ---
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_white = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 25, 255]))
        mask_yellow = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([30, 255, 255]))
        mask = cv2.bitwise_or(mask_white, mask_yellow)

        edges = cv2.Canny(mask, 50, 150)
        height, width = edges.shape
        roi = np.zeros_like(edges)
        polygon = np.array([[(0, height), (width, height), (width, height//2), (0, height//2)]], np.int32)
        cv2.fillPoly(roi, polygon, 255)
        masked_edges = cv2.bitwise_and(edges, roi)

        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=150)
        line_image = np.zeros_like(frame)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 3)

        result_frame = cv2.addWeighted(result_frame, 0.8, line_image, 1.0, 0)

        # --- 2. 전방 장애물 검출 (Hailo-8) ---
        if self.npu_ready:
            # TODO: 선택한 HEF 모델의 입력 사이즈에 맞게 프레임 리사이즈 후 추론
            # 예: input_data = cv2.resize(frame, (640, 640))
            # 텐서 출력값을 파싱하여 obstacle_detected = True/False 로 변환하는 후처리 로직 필요
            pass
        else:
            # 테스트용 임시 로직
            obstacle_detected = int(time.time()) % 10 < 3

        if obstacle_detected:
            cv2.putText(result_frame, "OBSTACLE DETECTED", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        return result_frame, obstacle_detected
