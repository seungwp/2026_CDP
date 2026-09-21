import cv2
import time
import mediapipe as mp

# --- 눈 랜드마크 인덱스 (EAR용 6점) ---
# 순서: [왼쪽끝, 위1, 위2, 오른쪽끝, 아래2, 아래1]
# EAR = (|p2-p6| + |p3-p5|) / (2 * |p1-p4|)
LEFT_EYE  = [33, 160, 158, 133, 153, 144]
RIGHT_EYE = [362, 385, 387, 263, 373, 380]

mp_face_mesh = mp.solutions.face_mesh
mp_drawing = mp.solutions.drawing_utils
mp_styles = mp.solutions.drawing_styles

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    raise SystemExit("[ERROR] 카메라를 열 수 없습니다.")

print("[INFO] 시작. q = 종료, m = 전체 메시 on/off")

show_mesh = True
prev = time.time()
fps = 0.0

with mp_face_mesh.FaceMesh(
    max_num_faces=1,
    refine_landmarks=True,      # 눈/입술 정밀 모드 (눈 정확도 올라감)
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
) as face_mesh:

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]

        # MediaPipe는 RGB를 받습니다. OpenCV는 BGR이라 변환 필요.
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False          # 성능 최적화
        results = face_mesh.process(rgb)

        if results.multi_face_landmarks:
            face = results.multi_face_landmarks[0]

            # 전체 메시 그리기 (m 키로 토글)
            if show_mesh:
                mp_drawing.draw_landmarks(
                    image=frame,
                    landmark_list=face,
                    connections=mp_face_mesh.FACEMESH_TESSELATION,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=mp_styles
                        .get_default_face_mesh_tesselation_style(),
                )

            # EAR용 6점만 강조 + 번호 표시
            for eye_name, idxs, color in [
                ("L", LEFT_EYE,  (0, 255, 0)),
                ("R", RIGHT_EYE, (0, 165, 255)),
            ]:
                for order, idx in enumerate(idxs, start=1):
                    lm = face.landmark[idx]
                    # 정규화 좌표(0~1) → 픽셀 좌표
                    x, y = int(lm.x * w), int(lm.y * h)
                    cv2.circle(frame, (x, y), 3, color, -1)
                    cv2.putText(frame, f"p{order}", (x + 4, y - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

            status = "FACE OK"
            status_color = (0, 255, 0)
        else:
            status = "NO FACE"
            status_color = (0, 0, 255)

        now = time.time()
        dt = now - prev
        prev = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)

        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(frame, status, (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

        cv2.imshow("Face Mesh Test", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('m'):
            show_mesh = not show_mesh

cap.release()
cv2.destroyAllWindows()
print(f"[INFO] 종료. 최종 FPS: {fps:.1f}")