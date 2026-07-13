import cv2
import os
import sys
from unittest.mock import MagicMock

# =====================================================================
# [핵심] ROS2 가짜(Mock) 모듈 주입
# 노트북 환경에는 ROS2(rclpy)가 없으므로, 파이썬이 에러를 내지 않고 
# 무사히 넘어가도록 가짜 객체(MagicMock)를 주입해 줍니다.
# =====================================================================
mock_module = MagicMock()
sys.modules['rclpy'] = mock_module
sys.modules['rclpy.node'] = mock_module
sys.modules['cv_bridge'] = mock_module
sys.modules['sensor_msgs'] = mock_module
sys.modules['sensor_msgs.msg'] = mock_module
sys.modules['std_msgs'] = mock_module
sys.modules['std_msgs.msg'] = mock_module
sys.modules['geometry_msgs'] = mock_module
sys.modules['geometry_msgs.msg'] = mock_module

# 반드시 위 가짜 모듈 세팅이 끝난 후에 임포트해야 에러가 나지 않습니다!
from vision_detector_node import VisionDetector

def main():
    # 영상 경로
    video_path = r"C:\Users\jinda\OneDrive\바탕 화면\2026_CDP\test_data\track_video.mp4"
    
    if not os.path.exists(video_path):
        print(f"오류: 영상을 찾을 수 없습니다. 경로를 확인하세요: {video_path}")
        return

    print("[System] 오프라인 비전 튜닝 모드 시작...")
    
    # ROS2 환경이 아니어도 가짜 노드로 인식되어 무사히 생성됩니다.
    detector = VisionDetector()
    cap = cv2.VideoCapture(video_path)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("영상 재생이 끝났습니다.")
            break
            
        # 640x480 해상도 고정
        frame = cv2.resize(frame, (640, 480)) 
        
        # 인지 로직 통과 (영상 처리만 단독 실행)
        debug_frame, offset = detector.process_frame(frame)
        
        # 화면 출력
        cv2.imshow("Offline Vision Tuning (Press ESC to exit)", debug_frame)
        
        # 30ms 대기 (ESC 키 누르면 강제 종료)
        if cv2.waitKey(30) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()