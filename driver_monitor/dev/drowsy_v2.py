import cv2
import time
import threading
import numpy as np
import mediapipe as mp
from collections import deque

try:
    import winsound
    HAS_SOUND = True
except ImportError:
    HAS_SOUND = False

# ===== 설정 =====
LEFT_EYE  = [33, 160, 158, 133, 153, 144]
RIGHT_EYE = [362, 385, 387, 263, 373, 380]

CALIB_SECONDS = 10.0
CLOSE_RATIO   = 0.60      # 감김 진입 기준 (baseline 대비)
OPEN_RATIO    = 0.70      # 뜸 복귀 기준  (히스테리시스 간격)

WARN_SEC      = 0.40
CRITICAL_SEC  = 1.00
BLINK_MAX     = 0.40

PERCLOS_WINDOW = 30.0
PERCLOS_WARN   = 15.0
PERCLOS_CRIT   = 25.0

DWELL_SEC      = 1.50     # 경고 최소 유지 시간
RECOVER_SEC    = 2.00

RANK = {"NORMAL": 0, "WARNING": 1, "CRITICAL": 2}
COLOR = {"NORMAL": (0, 255, 0), "WARNING": (0, 165, 255), "CRITICAL": (0, 0, 255)}


# ===== 비동기 알람 =====
class Alarm:
    """소리를 별도 스레드에서 재생. 메인 루프를 막지 않는다."""

    def __init__(self):
        self.level = "NORMAL"
        self.running = False
        self.thread = None
        self.lock = threading.Lock()

    def _loop(self):
        while True:
            with self.lock:
                if not self.running:
                    break
                lv = self.level
            if not HAS_SOUND:
                time.sleep(0.1)
                continue
            try:
                if lv == "CRITICAL":
                    winsound.Beep(1500, 150)   # 높고 빠른 연속음
                    time.sleep(0.08)
                elif lv == "WARNING":
                    winsound.Beep(800, 120)    # 낮고 느린 단속음
                    time.sleep(0.55)
                else:
                    time.sleep(0.05)
            except RuntimeError:
                time.sleep(0.1)

    def set_level(self, level):
        """매 프레임 호출해도 됨. 바뀔 때만 실제로 동작."""
        with self.lock:
            if level == self.level:
                return
            self.level = level
        if level == "NORMAL":
            self.running = False
        elif not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._loop, daemon=True)
            self.thread.start()

    def stop(self):
        with self.lock:
            self.running = False
            self.level = "NORMAL"


# ===== EAR =====
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


# ===== 메인 =====
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    raise SystemExit("[ERROR] 카메라를 열 수 없습니다.")

mp_face_mesh = mp.solutions.face_mesh
alarm = Alarm()

mode = "CALIB"
calib_samples, calib_start = [], None
baseline = thr_close = thr_open = None

eye_closed = False              # 히스테리시스로 관리되는 현재 눈 상태
closed_start, closed_dur = None, 0.0

state = "NORMAL"                # 최종 출력 상태 (dwell 적용 후)
raw_state = "NORMAL"            # dwell 적용 전 원본
state_since = time.time()       # 현재 state로 바뀐 시각
open_since = None
prev_state = "NORMAL"
cause = "-"

blink_count = warn_count = crit_count = 0
event_log = []                  # (시각, 이전상태, 새상태, EAR, PERCLOS, 원인)

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

        # ---------- 캘리브레이션 ----------
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
                    baseline  = float(np.percentile(np.array(calib_samples), 80))
                    thr_close = baseline * CLOSE_RATIO
                    thr_open  = baseline * OPEN_RATIO
                    mode = "RUN"
                    state_since = now
                    print(f"[CALIB] baseline  {baseline:.3f}")
                    print(f"[CALIB] close/open {thr_close:.3f} / {thr_open:.3f}")
            else:
                cv2.putText(frame, "NO FACE", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        # ---------- 실행 ----------
        else:
            if ear is None:
                cv2.putText(frame, "NO FACE", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                eye_closed = False
                closed_start, closed_dur = None, 0.0
                raw_state = "NORMAL"
            else:
                # ----- 히스테리시스 -----
                if eye_closed:
                    if ear > thr_open:
                        eye_closed = False
                else:
                    if ear < thr_close:
                        eye_closed = True

                # ----- PERCLOS -----
                if 0 < dt < 1.0:
                    window.append((now, eye_closed, dt))
                while window and now - window[0][0] > PERCLOS_WINDOW:
                    window.popleft()
                if window:
                    total = sum(x[2] for x in window)
                    closed_t = sum(x[2] for x in window if x[1])
                    perclos = (closed_t / total * 100.0) if total > 0 else 0.0
                    span = now - window[0][0]
                else:
                    perclos, span = 0.0, 0.0

                # ----- 순간 지표 -----
                if eye_closed:
                    if closed_start is None:
                        closed_start = now
                    closed_dur = now - closed_start
                    if closed_dur >= CRITICAL_SEC:
                        inst = "CRITICAL"
                    elif closed_dur >= WARN_SEC:
                        inst = "WARNING"
                    else:
                        inst = "NORMAL"
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
                    inst = "NORMAL"

                # ----- 누적 지표 -----
                if perclos >= PERCLOS_CRIT:
                    perc = "CRITICAL"
                elif perclos >= PERCLOS_WARN:
                    perc = "WARNING"
                else:
                    perc = "NORMAL"

                               # ----- 둘 중 심한 쪽 -----
                if RANK[inst] >= RANK[perc]:
                    raw_state, cause = inst, "INSTANT"
                else:
                    raw_state, cause = perc, "PERCLOS"

                # ----- 각성 복귀: 깜빡임은 리셋하지 않음 -----
                if not eye_closed:
                    if open_since is None:
                        open_since = now
                    if now - open_since >= RECOVER_SEC:
                        raw_state, cause = "NORMAL", "RECOVERED"
                else:
                    # 긴 감김(BLINK_MAX 초과)만 각성 타이머를 리셋
                    if closed_dur > BLINK_MAX:
                        open_since = None

            # ----- dwell: 악화는 즉시, 완화는 지연 -----
            if RANK[raw_state] > RANK[state]:
                state = raw_state
                state_since = now
            elif RANK[raw_state] < RANK[state]:
                # RECOVERED는 즉시 해제 (이미 2초 기다린 것)
                if cause == "RECOVERED" or now - state_since >= DWELL_SEC:
                    state = raw_state
                    state_since = now
                        # ----- 상태 전환 로그 -----
            if state != prev_state:
                ts = time.strftime('%H:%M:%S')
                e = (now, prev_state, state,
                     ear if ear else 0.0, perclos, cause)
                event_log.append(e)
                print(f"[{ts}] {prev_state} -> {state}  "
                      f"EAR {e[3]:.3f}  PERCLOS {perclos:.1f}%  by {cause}")
                prev_state = state

            alarm.set_level(state)

            # ----- 화면 -----
            c = COLOR[state]
            if state == "CRITICAL":
                cv2.rectangle(frame, (0, 0), (w - 1, h - 1), (0, 0, 255), 12)

            if ear is not None:
                cv2.putText(frame, f"EAR {ear:.3f}", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(frame, "CLOSED" if eye_closed else "OPEN",
                            (170, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 0, 255) if eye_closed else (0, 255, 0), 2)

            cv2.putText(frame, state, (10, 95),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, c, 3)
            if state != "NORMAL":
                held = now - state_since
                cv2.putText(frame, f"by {cause}   held {held:.1f}s",
                            (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1)
            cv2.putText(frame, f"closed {closed_dur:.2f}s", (10, 145),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            if ear is not None:
                # EAR 막대 + 히스테리시스 두 선
                bar = int(min(ear, 0.4) / 0.4 * 300)
                xc = int(min(thr_close, 0.4) / 0.4 * 300)
                xo = int(min(thr_open,  0.4) / 0.4 * 300)
                cv2.rectangle(frame, (10, 155), (310, 175), (80, 80, 80), 1)
                cv2.rectangle(frame, (10, 155), (10 + bar, 175), c, -1)
                cv2.line(frame, (10 + xc, 150), (10 + xc, 180), (255, 0, 255), 2)
                cv2.line(frame, (10 + xo, 150), (10 + xo, 180), (255, 255, 0), 2)

                # PERCLOS 막대
                pc = COLOR[perc if ear is not None else "NORMAL"]
                pbar = int(min(perclos, 30.0) / 30.0 * 300)
                cv2.rectangle(frame, (10, 195), (310, 215), (80, 80, 80), 1)
                cv2.rectangle(frame, (10, 195), (10 + pbar, 215), pc, -1)
                for mk, mc in [(PERCLOS_WARN, (0, 165, 255)),
                               (PERCLOS_CRIT, (0, 0, 255))]:
                    mx = int(mk / 30.0 * 300)
                    cv2.line(frame, (10 + mx, 190), (10 + mx, 220), mc, 1)
                cv2.putText(frame, f"PERCLOS {perclos:.1f}%  ({span:.0f}s)",
                            (10, 238), cv2.FONT_HERSHEY_SIMPLEX, 0.6, pc, 2)

            cv2.putText(frame,
                        f"blink {blink_count}  warn {warn_count}  crit {crit_count}"
                        f"   events {len(event_log)}",
                        (10, 262), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        prev = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
        cv2.putText(frame, f"FPS {fps:.1f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        cv2.imshow("Drowsiness Detection v2", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            alarm.stop()
            mode, calib_samples, calib_start = "CALIB", [], None
            window.clear()
            blink_count = warn_count = crit_count = 0
            event_log.clear()
            eye_closed = False
            state = raw_state = prev_state = "NORMAL"
            open_since = None
            print("[INFO] 재캘리브레이션")

alarm.stop()
cap.release()
cv2.destroyAllWindows()

print(f"\n[RESULT] blink {blink_count} / warn {warn_count} / crit {crit_count}")
print(f"[RESULT] 상태 전환 {len(event_log)}회, 최종 PERCLOS {perclos:.1f}%")