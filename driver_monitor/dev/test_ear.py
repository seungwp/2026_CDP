import cv2
import time
import numpy as np
import mediapipe as mp

LEFT_EYE  = [33, 160, 158, 133, 153, 144]
RIGHT_EYE = [362, 385, 387, 263, 373, 380]


def get_points(landmarks, idxs, w, h):
    """정규화 좌표(0~1) → 픽셀 좌표 numpy 배열"""
    return np.array([[landmarks[i].x * w, landmarks[i].y * h] for i in idxs])


def eye_aspect_ratio(pts):
    """
    pts: [p1, p2, p3, p4, p5, p6]  (인덱스 0~5)
    EAR = (|p2-p6| + |p3-p5|) / (2 * |p1-p4|)
    """
    vert1 = np.linalg.norm(pts[1] - pts[5])   # p2 - p6
    vert2 = np.linalg.norm(pts[2] - pts[4])   # p3 - p5
    horiz = np.linalg.norm(pts[0] - pts[3])   # p1 - p4
    if horiz < 1e-6:                          # 0으로 나누기 방지
        return 0.0
    return (vert1 + vert2) / (2.0 * horiz)


mp_face_mesh = mp.solutions.face_mesh

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    raise SystemExit("[ERROR] 카메라를 열 수 없습니다.")

print("[INFO] 시작. q = 종료")
print("[INFO] 눈을 뜬 값과 감은 값을 기록해두세요.")

prev = time.time()
fps = 0.0

# 관찰용 최소/최대 기록
ear_min, ear_max = 1.0, 0.0

with mp_face_mesh.FaceMesh(
    max_num_faces=1,
    refine_landmarks=True,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
) as face_mesh:

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = face_mesh.process(rgb)

        ear = None

        if results.multi_face_landmarks:
            lms = results.multi_face_landmarks[0].landmark

            left_pts  = get_points(lms, LEFT_EYE,  w, h)
            right_pts = get_points(lms, RIGHT_EYE, w, h)

            left_ear  = eye_aspect_ratio(left_pts)
            right_ear = eye_aspect_ratio(right_pts)
            ear = (left_ear + right_ear) / 2.0      # 양쪽 평균

            ear_min = min(ear_min, ear)
            ear_max = max(ear_max, ear)

            # 눈 6점 표시 (디버깅용)
            for pts, color in [(left_pts, (0, 255, 0)), (right_pts, (0, 165, 255))]:
                for (x, y) in pts.astype(int):
                    cv2.circle(frame, (x, y), 2, color, -1)

            # EAR 숫자 표시 (크게)
            cv2.putText(frame, f"EAR: {ear:.3f}", (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
            cv2.putText(frame, f"L {left_ear:.3f}  R {right_ear:.3f}", (10, 135),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

            # 막대 그래프 (0 ~ 0.4 범위를 300px에 매핑)
            bar_len = int(min(ear, 0.4) / 0.4 * 300)
            cv2.rectangle(frame, (10, 150), (310, 175), (80, 80, 80), 1)
            cv2.rectangle(frame, (10, 150), (10 + bar_len, 175), (0, 255, 255), -1)

            cv2.putText(frame, f"min {ear_min:.3f}  max {ear_max:.3f}", (10, 200),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
        else:
            cv2.putText(frame, "NO FACE", (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        now = time.time()
        dt = now - prev
        prev = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        cv2.imshow("EAR Test", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
print(f"[INFO] 종료. FPS {fps:.1f} / EAR 범위 {ear_min:.3f} ~ {ear_max:.3f}")