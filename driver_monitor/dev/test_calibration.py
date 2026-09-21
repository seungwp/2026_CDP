import cv2
import time
import numpy as np
import mediapipe as mp

LEFT_EYE  = [33, 160, 158, 133, 153, 144]
RIGHT_EYE = [362, 385, 387, 263, 373, 380]

CALIB_SECONDS = 5.0       # 캘리브레이션 측정 시간
CLOSE_RATIO   = 0.70      # 기준선의 70% 아래면 "감김"으로 판정


def get_points(landmarks, idxs, w, h):
    return np.array([[landmarks[i].x * w, landmarks[i].y * h] for i in idxs])


def eye_aspect_ratio(pts):
    vert1 = np.linalg.norm(pts[1] - pts[5])
    vert2 = np.linalg.norm(pts[2] - pts[4])
    horiz = np.linalg.norm(pts[0] - pts[3])
    if horiz < 1e-6:
        return 0.0
    return (vert1 + vert2) / (2.0 * horiz)


def compute_ear(lms, w, h):
    l = eye_aspect_ratio(get_points(lms, LEFT_EYE,  w, h))
    r = eye_aspect_ratio(get_points(lms, RIGHT_EYE, w, h))
    return (l + r) / 2.0


mp_face_mesh = mp.solutions.face_mesh

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    raise SystemExit("[ERROR] 카메라를 열 수 없습니다.")

# --- 상태 ---
mode = "CALIB"            # CALIB -> RUN
calib_samples = []
calib_start = None
baseline = None
threshold = None

print("[INFO] 캘리브레이션을 시작합니다. 화면을 정면으로 보세요.")
print("[INFO] q = 종료, r = 캘리브레이션 다시")

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
            ear = compute_ear(results.multi_face_landmarks[0].landmark, w, h)

        # ---------- 캘리브레이션 ----------
        if mode == "CALIB":
            if ear is not None:
                if calib_start is None:
                    calib_start = time.time()
                calib_samples.append(ear)

                elapsed = time.time() - calib_start
                remain = max(0.0, CALIB_SECONDS - elapsed)

                cv2.putText(frame, "CALIBRATING", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                cv2.putText(frame, "Look at the camera. Blink normally.",
                            (10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                cv2.putText(frame, f"{remain:.1f}s", (10, 135),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

                # 진행 바
                prog = int(min(elapsed / CALIB_SECONDS, 1.0) * 300)
                cv2.rectangle(frame, (10, 150), (310, 170), (80, 80, 80), 1)
                cv2.rectangle(frame, (10, 150), (10 + prog, 170), (0, 255, 255), -1)

                if elapsed >= CALIB_SECONDS:
                    arr = np.array(calib_samples)
                    # 깜빡임이 섞여 있으므로 상위 구간만 사용 -> "뜬 눈" 기준선
                    baseline = float(np.percentile(arr, 75))
                    threshold = baseline * CLOSE_RATIO
                    mode = "RUN"
                    print(f"[CALIB] 샘플 {len(arr)}개")
                    print(f"[CALIB] baseline(75%) = {baseline:.3f}")
                    print(f"[CALIB] threshold     = {threshold:.3f}")
            else:
                cv2.putText(frame, "NO FACE - show your face", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # ---------- 실행 ----------
        else:
            if ear is not None:
                closed = ear < threshold
                color = (0, 0, 255) if closed else (0, 255, 0)
                label = "CLOSED" if closed else "OPEN"

                cv2.putText(frame, f"EAR {ear:.3f}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                cv2.putText(frame, label, (10, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)

                # 막대 + 임계선
                bar = int(min(ear, 0.4) / 0.4 * 300)
                thr_x = int(min(threshold, 0.4) / 0.4 * 300)
                cv2.rectangle(frame, (10, 120), (310, 145), (80, 80, 80), 1)
                cv2.rectangle(frame, (10, 120), (10 + bar, 145), color, -1)
                cv2.line(frame, (10 + thr_x, 115), (10 + thr_x, 150),
                         (255, 0, 255), 2)

                cv2.putText(frame,
                            f"base {baseline:.3f}  thr {threshold:.3f}",
                            (10, 170), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (180, 180, 180), 1)
            else:
                cv2.putText(frame, "NO FACE", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        cv2.imshow("Calibration Test", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            mode, calib_samples, calib_start = "CALIB", [], None
            print("[INFO] 캘리브레이션 재시작")

cap.release()
cv2.destroyAllWindows()