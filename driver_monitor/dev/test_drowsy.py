import cv2
import time
import numpy as np
import mediapipe as mp

try:
    import winsound
    HAS_SOUND = True
except ImportError:
    HAS_SOUND = False

LEFT_EYE  = [33, 160, 158, 133, 153, 144]
RIGHT_EYE = [362, 385, 387, 263, 373, 380]

CALIB_SECONDS = 5.0
CLOSE_RATIO   = 0.70

WARN_SEC     = 0.40      # 이 이상 감겨 있으면 주의
CRITICAL_SEC = 1.00      # 이 이상이면 경보
BLINK_MAX    = 0.40      # 이 이하는 정상 깜빡임으로 집계


def get_points(lms, idxs, w, h):
    return np.array([[lms[i].x * w, lms[i].y * h] for i in idxs])


def eye_aspect_ratio(pts):
    v1 = np.linalg.norm(pts[1] - pts[5])
    v2 = np.linalg.norm(pts[2] - pts[4])
    hz = np.linalg.norm(pts[0] - pts[3])
    return 0.0 if hz < 1e-6 else (v1 + v2) / (2.0 * hz)


def compute_ear(lms, w, h):
    l = eye_aspect_ratio(get_points(lms, LEFT_EYE,  w, h))
    r = eye_aspect_ratio(get_points(lms, RIGHT_EYE, w, h))
    return (l + r) / 2.0


def beep(level):
    if not HAS_SOUND:
        return
    try:
        if level == "CRITICAL":
            winsound.Beep(1500, 120)
        elif level == "WARNING":
            winsound.Beep(800, 80)
    except RuntimeError:
        pass


mp_face_mesh = mp.solutions.face_mesh

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    raise SystemExit("[ERROR] 카메라를 열 수 없습니다.")

mode = "CALIB"
calib_samples, calib_start = [], None
baseline = threshold = None

closed_start = None       # 눈 감기 시작한 시각
closed_dur   = 0.0
state        = "NORMAL"
blink_count  = 0
warn_count   = 0
crit_count   = 0
last_beep    = 0.0

prev, fps = time.time(), 0.0

print("[INFO] 캘리브레이션 시작. q = 종료, r = 재캘리브레이션")

with mp_face_mesh.FaceMesh(
    max_num_faces=1, refine_landmarks=True,
    min_detection_confidence=0.5, min_tracking_confidence=0.5,
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

        now = time.time()

        # ---------- 캘리브레이션 ----------
        if mode == "CALIB":
            if ear is not None:
                if calib_start is None:
                    calib_start = now
                calib_samples.append(ear)
                elapsed = now - calib_start
                remain = max(0.0, CALIB_SECONDS - elapsed)

                cv2.putText(frame, "CALIBRATING", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                cv2.putText(frame, f"{remain:.1f}s", (10, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

                if elapsed >= CALIB_SECONDS:
                    baseline = float(np.percentile(np.array(calib_samples), 75))
                    threshold = baseline * CLOSE_RATIO
                    mode = "RUN"
                    print(f"[CALIB] baseline {baseline:.3f} / threshold {threshold:.3f}")
            else:
                cv2.putText(frame, "NO FACE", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        # ---------- 실행 ----------
        else:
            if ear is None:
                # 얼굴이 사라진 것도 위험 신호 (고개 숙임 등)
                cv2.putText(frame, "NO FACE", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                closed_start, closed_dur, state = None, 0.0, "NORMAL"
            else:
                is_closed = ear < threshold

                if is_closed:
                    if closed_start is None:
                        closed_start = now          # 감기 시작
                    closed_dur = now - closed_start

                    if closed_dur >= CRITICAL_SEC:
                        state = "CRITICAL"
                    elif closed_dur >= WARN_SEC:
                        state = "WARNING"
                    else:
                        state = "NORMAL"            # 아직 깜빡임 범위
                else:
                    # 눈을 떴다 -> 방금 끝난 구간 집계
                    if closed_start is not None:
                        dur = now - closed_start
                        if dur <= BLINK_MAX:
                            blink_count += 1
                        elif dur < CRITICAL_SEC:
                            warn_count += 1
                        else:
                            crit_count += 1
                    closed_start, closed_dur, state = None, 0.0, "NORMAL"

                # ----- 표시 -----
                color = {"NORMAL": (0, 255, 0),
                         "WARNING": (0, 165, 255),
                         "CRITICAL": (0, 0, 255)}[state]

                if state == "CRITICAL":
                    cv2.rectangle(frame, (0, 0), (w - 1, h - 1), (0, 0, 255), 12)

                cv2.putText(frame, f"EAR {ear:.3f}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(frame, state, (10, 105),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.3, color, 3)
                cv2.putText(frame, f"closed {closed_dur:.2f}s", (10, 140),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                bar = int(min(ear, 0.4) / 0.4 * 300)
                thr = int(min(threshold, 0.4) / 0.4 * 300)
                cv2.rectangle(frame, (10, 155), (310, 178), (80, 80, 80), 1)
                cv2.rectangle(frame, (10, 155), (10 + bar, 178), color, -1)
                cv2.line(frame, (10 + thr, 150), (10 + thr, 183), (255, 0, 255), 2)

                cv2.putText(frame,
                            f"blink {blink_count}  warn {warn_count}  crit {crit_count}",
                            (10, 205), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

                # ----- 소리 (0.5초 간격 제한) -----
                if state in ("WARNING", "CRITICAL") and now - last_beep > 0.5:
                    beep(state)
                    last_beep = now

        dt = now - prev
        prev = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
        cv2.putText(frame, f"FPS {fps:.1f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        cv2.imshow("Drowsiness Detection", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            mode, calib_samples, calib_start = "CALIB", [], None
            blink_count = warn_count = crit_count = 0
            print("[INFO] 재캘리브레이션")

cap.release()
cv2.destroyAllWindows()
print(f"[RESULT] 깜빡임 {blink_count} / 주의 {warn_count} / 경보 {crit_count}")