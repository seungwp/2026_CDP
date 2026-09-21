import cv2
import time
import numpy as np
import mediapipe as mp
from collections import deque

try:
    import winsound
    HAS_SOUND = True
except ImportError:
    HAS_SOUND = False

LEFT_EYE  = [33, 160, 158, 133, 153, 144]
RIGHT_EYE = [362, 385, 387, 263, 373, 380]

CALIB_SECONDS = 5.0
CLOSE_RATIO   = 0.70

WARN_SEC     = 0.40
CRITICAL_SEC = 1.00
BLINK_MAX    = 0.40

PERCLOS_WINDOW = 60.0     # 관측 창 (초)
PERCLOS_WARN   = 8.0      # %
PERCLOS_CRIT   = 15.0     # %


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
        winsound.Beep(1500 if level == "CRITICAL" else 800,
                      120 if level == "CRITICAL" else 80)
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

closed_start, closed_dur = None, 0.0
state = "NORMAL"
blink_count = warn_count = crit_count = 0
last_beep = 0.0

# PERCLOS용: (시각, 감김여부, 직전프레임과의 간격)
window = deque()
perclos = 0.0

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
        dt = now - prev

        if mode == "CALIB":
            if ear is not None:
                if calib_start is None:
                    calib_start = now
                calib_samples.append(ear)
                elapsed = now - calib_start
                cv2.putText(frame, "CALIBRATING", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                cv2.putText(frame, f"{max(0.0, CALIB_SECONDS - elapsed):.1f}s",
                            (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                if elapsed >= CALIB_SECONDS:
                    baseline = float(np.percentile(np.array(calib_samples), 75))
                    threshold = baseline * CLOSE_RATIO
                    mode = "RUN"
                    print(f"[CALIB] baseline {baseline:.3f} / threshold {threshold:.3f}")
            else:
                cv2.putText(frame, "NO FACE", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        else:
            if ear is None:
                cv2.putText(frame, "NO FACE", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                closed_start, closed_dur, state = None, 0.0, "NORMAL"
            else:
                is_closed = ear < threshold

                # ----- PERCLOS 슬라이딩 윈도우 -----
                if 0 < dt < 1.0:                      # 비정상 간격 제외
                    window.append((now, is_closed, dt))
                while window and now - window[0][0] > PERCLOS_WINDOW:
                    window.popleft()

                if window:
                    total = sum(item[2] for item in window)
                    closed_t = sum(item[2] for item in window if item[1])
                    perclos = (closed_t / total * 100.0) if total > 0 else 0.0
                    span = now - window[0][0]
                else:
                    perclos, span = 0.0, 0.0

                # ----- 순간 감김 판정 -----
                if is_closed:
                    if closed_start is None:
                        closed_start = now
                    closed_dur = now - closed_start
                    if closed_dur >= CRITICAL_SEC:
                        inst_state = "CRITICAL"
                    elif closed_dur >= WARN_SEC:
                        inst_state = "WARNING"
                    else:
                        inst_state = "NORMAL"
                else:
                    if closed_start is not None:
                        d = now - closed_start
                        if d <= BLINK_MAX:
                            blink_count += 1
                        elif d < CRITICAL_SEC:
                            warn_count += 1
                        else:
                            crit_count += 1
                    closed_start, closed_dur = None, 0.0
                    inst_state = "NORMAL"

                # ----- PERCLOS 판정 -----
                if perclos >= PERCLOS_CRIT:
                    perc_state = "CRITICAL"
                elif perclos >= PERCLOS_WARN:
                    perc_state = "WARNING"
                else:
                    perc_state = "NORMAL"

                # ----- 최종: 둘 중 심한 쪽 -----
                rank = {"NORMAL": 0, "WARNING": 1, "CRITICAL": 2}
                state = inst_state if rank[inst_state] >= rank[perc_state] else perc_state
                cause = "INSTANT" if rank[inst_state] >= rank[perc_state] else "PERCLOS"

                color = {"NORMAL": (0, 255, 0),
                         "WARNING": (0, 165, 255),
                         "CRITICAL": (0, 0, 255)}[state]

                if state == "CRITICAL":
                    cv2.rectangle(frame, (0, 0), (w - 1, h - 1), (0, 0, 255), 12)

                cv2.putText(frame, f"EAR {ear:.3f}", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(frame, state, (10, 95),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
                if state != "NORMAL":
                    cv2.putText(frame, f"by {cause}", (10, 120),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

                cv2.putText(frame, f"closed {closed_dur:.2f}s", (10, 145),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

                # EAR 막대
                bar = int(min(ear, 0.4) / 0.4 * 300)
                thr = int(min(threshold, 0.4) / 0.4 * 300)
                cv2.rectangle(frame, (10, 155), (310, 175), (80, 80, 80), 1)
                cv2.rectangle(frame, (10, 155), (10 + bar, 175), color, -1)
                cv2.line(frame, (10 + thr, 150), (10 + thr, 180), (255, 0, 255), 2)

                # PERCLOS 막대 (0~30% 범위)
                pcolor = {"NORMAL": (0, 255, 0),
                          "WARNING": (0, 165, 255),
                          "CRITICAL": (0, 0, 255)}[perc_state]
                pbar = int(min(perclos, 30.0) / 30.0 * 300)
                cv2.rectangle(frame, (10, 195), (310, 215), (80, 80, 80), 1)
                cv2.rectangle(frame, (10, 195), (10 + pbar, 215), pcolor, -1)
                for mark, mc in [(PERCLOS_WARN, (0, 165, 255)),
                                 (PERCLOS_CRIT, (0, 0, 255))]:
                    mx = int(mark / 30.0 * 300)
                    cv2.line(frame, (10 + mx, 190), (10 + mx, 220), mc, 1)

                cv2.putText(frame, f"PERCLOS {perclos:.1f}%  ({span:.0f}s)",
                            (10, 238), cv2.FONT_HERSHEY_SIMPLEX, 0.6, pcolor, 2)
                cv2.putText(frame,
                            f"blink {blink_count}  warn {warn_count}  crit {crit_count}",
                            (10, 262), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

                if state in ("WARNING", "CRITICAL") and now - last_beep > 0.5:
                    beep(state)
                    last_beep = now

        prev = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
        cv2.putText(frame, f"FPS {fps:.1f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        cv2.imshow("Drowsiness Detection (PERCLOS)", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            mode, calib_samples, calib_start = "CALIB", [], None
            window.clear()
            blink_count = warn_count = crit_count = 0
            print("[INFO] 재캘리브레이션")

cap.release()
cv2.destroyAllWindows()
print(f"[RESULT] blink {blink_count} / warn {warn_count} / crit {crit_count} / PERCLOS {perclos:.1f}%")