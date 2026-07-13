import cv2
import numpy as np

class VisionDetector:
    def __init__(self):
        # 차선 인식을 위한 HSV 색상 범위 (트랙 환경에 맞게 튜닝 필요)
        # 흰색 차선
        self.lower_white = np.array([0, 0, 180])
        self.upper_white = np.array([180, 40, 255])
        # 노란색 차선
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([40, 255, 255])

    def process_image(self, frame):
        """
        카메라 프레임을 받아 차선 오프셋과 디버그용 이미지를 반환합니다.
        offset: -1.0(왼쪽 끝) ~ 0.0(중앙) ~ 1.0(오른쪽 끝)
        차선을 찾지 못한 경우 None을 반환합니다.
        """
        height, width = frame.shape[:2]
        
        # 1. 연산 속도 확보를 위해 이미지 하단(ROI)만 잘라서 사용
        roi_top = int(height * 0.5)
        roi = frame[roi_top:height, 0:width]
        roi_h, roi_w = roi.shape[:2]

        # 2. 가우시안 블러 및 HSV 변환
        blurred = cv2.GaussianBlur(roi, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 3. 색상 마스크 생성 (흰색 + 노란색)
        mask_white = cv2.inRange(hsv, self.lower_white, self.upper_white)
        mask_yellow = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        mask_combined = cv2.bitwise_or(mask_white, mask_yellow)

        # 4. 무게중심(Centroid) 계산을 통한 차선 중앙 파악
        M = cv2.moments(mask_combined)
        
        center_x = roi_w // 2
        
        if M["m00"] > 0:
            # 차선 픽셀들의 무게중심 X 좌표
            cx = int(M["m10"] / M["m00"])
            
            # 오프셋 계산: (차선 중심 - 화면 중심) / (화면 절반 너비)
            # 결과: 오른쪽으로 치우치면 양수(+), 왼쪽이면 음수(-)
            offset = (cx - center_x) / float(center_x)
            
            # 디버그 영상에 시각화 (인식된 중심점 파란색 선)
            cv2.line(roi, (cx, 0), (cx, roi_h), (255, 0, 0), 3)
        else:
            # [핵심 수정] 차선을 찾지 못했을 때 0.0이 아닌 None 반환
            offset = None

        # 디버그 영상 시각화 보강 (화면 중심 빨간색 선)
        cv2.line(roi, (center_x, 0), (center_x, roi_h), (0, 0, 255), 1)
        
        # 잘라냈던 ROI를 다시 원본 프레임에 덮어씌움
        debug_frame = frame.copy()
        debug_frame[roi_top:height, 0:width] = roi

        # [핵심 수정] offset이 None이 아닐 때만 범위 클리핑 (-1.0 ~ 1.0)
        if offset is not None:
            offset = max(-1.0, min(1.0, offset))

        return offset, debug_frame