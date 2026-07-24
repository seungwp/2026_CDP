import cv2
import numpy as np


class VisionDetector:
    """카메라 프레임에서 차선을 인식해 차로 중심 대비 횡방향 오프셋을 계산한다. (인지부, ROS 비의존 순수 로직)

    장애물 인식은 이 클래스가 아니라 Hailo NPU 노드(stella_hailo_rpi5_ros2_examples)가 담당한다.

    오프셋 규약: -1.0 ~ +1.0 정규화 값.
    + 는 차로 중심이 화면 중앙보다 오른쪽에 있음(차가 왼쪽으로 치우침 → 우조향 필요),
    - 는 그 반대. 차선을 하나도 못 찾으면 None.
    """

    # 튜닝 파라미터 — 테스트 트랙(테이프 색·조명·카메라 각도)에 맞춰 조정할 것
    # 2026-07-08 실측 기준: 카메라가 거의 수평이라 테이프가 화면 중간(0.45~0.7)에 보이고
    # 근접 차선은 좌우로 화면을 벗어난다. 카메라를 아래로 숙여 달면 이 값들 재튜닝 필요.
    ROI_TOP = 0.45        # 화면 높이의 이 비율 지점부터 아래에서만 차선 탐색
    Y_EVAL = 0.7          # 오프셋을 계산하는 기준 행(높이 비율, 테이프가 보이는 구간 안)
    MIN_ABS_SLOPE = 0.3   # 이보다 완만한(수평에 가까운) 선분은 차선으로 안 봄 (정지선/그림자 배제)
    HALF_LANE_PX = 340    # 한쪽 차선만 보일 때 가정하는 차로 반폭(픽셀, y_eval 행 기준 실측 근사)
    USE_WHITE = False     # 광택 바닥의 조명 반사가 흰색 마스크에 잡혀 가짜 차선을 만든다.
                          # 현재 트랙은 노란 테이프뿐이라 끔. 흰 테이프를 추가하면 켜고 HSV 재조정.

    def __init__(self):
        print("[System] Vision: OpenCV 차선 인식 초기화 완료.")

    def process_frame(self, frame):
        """(디버그 프레임, 차로 중심 오프셋 float 또는 None)을 반환한다."""
        height, width = frame.shape[:2]

        # 1. 차선 색 마스크 (노란색 범위는 실측 테이프 기준으로 여유 있게)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([18, 70, 70]), np.array([40, 255, 255]))
        if self.USE_WHITE:
            mask_white = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 25, 255]))
            mask = cv2.bitwise_or(mask, mask_white)

        # 2. 하단 ROI에서만 엣지/직선 검출
        edges = cv2.Canny(mask, 50, 150)
        roi_top = int(height * self.ROI_TOP)
        roi = np.zeros_like(edges)
        cv2.rectangle(roi, (0, roi_top), (width, height), 255, -1)
        masked_edges = cv2.bitwise_and(edges, roi)

        lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 50,
                                minLineLength=40, maxLineGap=120)

        debug = frame.copy()
        cv2.line(debug, (width // 2, roi_top), (width // 2, height), (255, 0, 0), 1)

        # 3. 각 선분을 기준 행(y_eval)까지 연장한 x좌표로 좌/우 차선 분류
        y_eval = int(height * self.Y_EVAL)
        left_xs, right_xs = [], []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                dx, dy = x2 - x1, y2 - y1
                if dx == 0:
                    x_at = float(x1)  # 수직선
                else:
                    slope = dy / dx
                    if abs(slope) < self.MIN_ABS_SLOPE:
                        continue
                    x_at = x1 + (y_eval - y1) / slope
                # 근접 차선은 기준 행에서 화면 밖으로 나가는 게 정상(카메라가 낮아서).
                # 화면 폭의 ±1배까지는 유효한 차선으로 인정하고, 그 이상만 노이즈로 버린다.
                if not (-width <= x_at < 2 * width):
                    continue
                cv2.line(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
                if x_at < width / 2:
                    left_xs.append(x_at)
                else:
                    right_xs.append(x_at)

        # 4. 차로 중심 추정 (한쪽만 보이면 반폭 가정으로 보정)
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
