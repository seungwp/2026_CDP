"""
drowsy_v4.py  -  Euro NCAP 구조 반영 버전

v3 대비 변경점
1. 경보를 두 채널로 분리
   [이벤트 채널]  긴급 / 경고음 / 눈감김 지속시간 기반
       MICROSLEEP    1초 이상   (Euro NCAP: 1-2s short eye closure)
       SLEEP         3초 이상   (Euro NCAP: continued eye closure >= 3s)
       UNRESPONSIVE  13초 이상  (Euro NCAP 무반응 운전자 시험 기준) -> 차량 개입
   [졸음 채널]    비긴급 / 화면 안내 + 차임 1회 / 누적 지표 기반
       DROWSY  <- PERCLOS 또는 최근 5분 마이크로슬립 누적
       대응: "휴식 권고" (Euro NCAP: 졸음은 휴게소 안내 등으로 대응)
2. 차량 제어 인터페이스 모킹 (VehicleInterface)
3. UNRESPONSIVE는 래치: 운전자가 C 키(= 조작 입력)를 눌러야 해제

키: q = 종료, r = 재캘리브레이션, c = 차량 개입 취소(운전자 조작)
"""

import csv
import os
import time
import threading
from collections import deque
from datetime import datetime

import cv2
import numpy as np
import mediapipe as mp

try:
    import winsound
    HAS_SOUND = True
except ImportError:
    HAS_SOUND = False


# ===== 설정 =====
LOG_DIR = "logs"

LEFT_EYE  = [33, 160, 158, 133, 153, 144]
RIGHT_EYE = [362, 385, 387, 263, 373, 380]

# --- 캘리브레이션 / 눈 상태 ---
CALIB_SECONDS    = 8.0
CALIB_PERCENTILE = 80
CLOSE_RATIO      = 0.60     # 감김 진입 (baseline 대비)
OPEN_RATIO       = 0.70     # 뜸 복귀 (히스테리시스)

# --- 이벤트 채널 (Euro NCAP 기준) ---
BLINK_MAX        = 0.40     # 이하 = 정상 깜빡임
MICROSLEEP_SEC   = 1.0      # 마이크로슬립
SLEEP_SEC        = 3.0      # 수면
UNRESPONSIVE_SEC = 13.0     # 무반응 운전자 -> 차량 개입
EVENT_DWELL_SEC  = 1.5      # 눈 뜬 뒤 경고 최소 유지

# --- 졸음 채널 (누적 지표, 비긴급) ---
PERCLOS_WINDOW   = 60.0     # 관측 창 (초)
PERCLOS_MIN_SPAN = 30.0     # 이만큼 쌓여야 판정 시작
PERCLOS_ON       = 15.0     # % 진입
PERCLOS_OFF      = 12.0     # % 해제 (히스테리시스)
MS_WINDOW        = 300.0    # 최근 5분
MS_DROWSY_COUNT  = 2        # 최근 5분 마이크로슬립 이 횟수 이상이면 졸음

AUTO_STOP_SEC    = 0        # 0 = 수동 종료(q)

EV_RANK  = {"NORMAL": 0, "MICROSLEEP": 1, "SLEEP": 2, "UNRESPONSIVE": 3}
EV_COLOR = {"NORMAL": (0, 255, 0), "MICROSLEEP": (0, 165, 255),
            "SLEEP": (0, 0, 255), "UNRESPONSIVE": (255, 0, 255)}


# ===== 경고음 =====
class Alarm:
    """이벤트 채널 경고음. 스레드 하나가 계속 돌며 현재 level에 맞는 소리를 냄."""

    PATTERN = {                        # (주파수 Hz, 길이 ms, 쉬는 시간 s)
        "MICROSLEEP":   (1000, 150, 0.45),
        "SLEEP":        (1500, 150, 0.08),
        "UNRESPONSIVE": (2000, 300, 0.05),
    }

    def __init__(self):
        self.level = "NORMAL"
        self.lock = threading.Lock()
        self.alive = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.alive:
            with self.lock:
                lv = self.level
            pat = self.PATTERN.get(lv)
            if pat is None or not HAS_SOUND:
                time.sleep(0.05)
                continue
            try:
                winsound.Beep(pat[0], pat[1])
            except RuntimeError:
                pass
            time.sleep(pat[2])

    def set_level(self, level):
        with self.lock:
            self.level = level

    def chime(self):
        """졸음 채널용 1회성 안내음."""
        def _play():
            if not HAS_SOUND:
                return
            try:
                winsound.Beep(600, 200)
                time.sleep(0.1)
                winsound.Beep(800, 300)
            except RuntimeError:
                pass
        threading.Thread(target=_play, daemon=True).start()

    def stop(self):
        with self.lock:
            self.level = "NORMAL"
        self.alive = False


# ===== 로깅 =====
class Logger:
    """프레임 단위 원시 데이터와 이벤트를 CSV로 기록."""

    def __init__(self, tag=""):
        os.makedirs(LOG_DIR, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        suffix = f"_{tag}" if tag else ""

        self.frame_path = os.path.join(LOG_DIR, f"frames_{stamp}{suffix}.csv")
        self.event_path = os.path.join(LOG_DIR, f"events_{stamp}{suffix}.csv")

        self.ff = open(self.frame_path, "w", newline="", encoding="utf-8")
        self.ef = open(self.event_path, "w", newline="", encoding="utf-8")
        self.fw = csv.writer(self.ff)
        self.ew = csv.writer(self.ef)

        self.fw.writerow([
            "elapsed_s", "fps", "ear", "eye_closed", "closed_dur_s",
            "perclos", "event_state", "drowsy", "face_detected",
        ])
        self.ew.writerow([
            "elapsed_s", "timestamp", "from_state", "to_state",
            "ear", "perclos", "cause", "held_s",
        ])

        self.t0 = time.time()
        self.frame_n = 0
        self.event_n = 0

    def log_frame(self, fps, ear, eye_closed, closed_dur,
                  perclos, ev_state, drowsy, face_ok):
        self.fw.writerow([
            f"{time.time() - self.t0:.3f}",
            f"{fps:.1f}",
            f"{ear:.4f}" if ear is not None else "",
            int(eye_closed),
            f"{closed_dur:.3f}",
            f"{perclos:.2f}",
            ev_state,
            int(drowsy),
            int(face_ok),
        ])
        self.frame_n += 1

    def log_event(self, from_s, to_s, ear, perclos, cause, held):
        self.ew.writerow([
            f"{time.time() - self.t0:.3f}",
            datetime.now().strftime("%H:%M:%S.%f")[:-3],
            from_s, to_s,
            f"{ear:.4f}" if ear is not None else "",
            f"{perclos:.2f}",
            cause,
            f"{held:.2f}",
        ])
        self.ef.flush()
        self.event_n += 1

    def close(self):
        self.ff.close()
        self.ef.close()
        print(f"\n[LOG] 프레임 {self.frame_n}행 -> {self.frame_path}")
        print(f"[LOG] 이벤트 {self.event_n}행 -> {self.event_path}")


# ===== 차량 제어 인터페이스 (모킹) =====
class VehicleInterface:
    """
    실제 차량에서는 CAN 메시지 등으로 제어기에 전달될 자리.
    여기서는 콘솔 출력 + 이벤트 로그로 대체한다.
    """

    def __init__(self, logger):
        self.logger = logger
        self.history = []

    def send(self, cmd, reason, ear=None, perclos=0.0):
        ts = datetime.now().strftime("%H:%M:%S")
        print(f"[{ts}] [VEHICLE] {cmd:<24} ({reason})")
        self.history.append((time.time(), cmd, reason))
        self.logger.log_event("VEHICLE", cmd, ear, perclos, reason, 0.0)


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


# ===== 판정 도우미 =====
def event_level(closed_dur):
    """현재 감고 있는 시간 -> 이벤트 단계."""
    if closed_dur >= UNRESPONSIVE_SEC:
        return "UNRESPONSIVE"
    if closed_dur >= SLEEP_SEC:
        return "SLEEP"
    if closed_dur >= MICROSLEEP_SEC:
        return "MICROSLEEP"
    return "NORMAL"


def classify_closure(d):
    """끝난 눈감김 1회 -> 분류."""
    if d <= BLINK_MAX:
        return "blink"
    if d < MICROSLEEP_SEC:
        return "long"
    if d < SLEEP_SEC:
        return "micro"
    if d < UNRESPONSIVE_SEC:
        return "sleep"
    return "unresp"


# ===== 메인 =====
SESSION_TAG = input("테스트 조건 태그 (예: ncap_micro, ncap_sleep / 엔터=생략): ").strip()
logger  = Logger(SESSION_TAG)
alarm   = Alarm()
vehicle = VehicleInterface(logger)

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    raise SystemExit("[ERROR] 카메라를 열 수 없습니다.")

mp_face_mesh = mp.solutions.face_mesh

# --- 캘리브레이션 ---
mode = "CALIB"
calib_samples, calib_start = [], None
baseline = thr_close = thr_open = None

# --- 눈 상태 ---
eye_closed = False
closed_start, closed_dur = None, 0.0

# --- 이벤트 채널 ---
ev_state = "NORMAL"
ev_since = time.time()
lower_since = None

# --- 졸음 채널 ---
drowsy = False
drowsy_cause = "-"
window = deque()            # (시각, 감김여부, dt)
perclos, span = 0.0, 0.0
ms_times = deque()          # 마이크로슬립 이상 눈감김이 끝난 시각들
ms_recent = 0

# --- 통계 ---
counts = {"blink": 0, "long": 0, "micro": 0, "sleep": 0, "unresp": 0}
ev_entries = {"MICROSLEEP": 0, "SLEEP": 0, "UNRESPONSIVE": 0}
drowsy_entries = 0

prev, fps = time.time(), 0.0
ear = None
now = time.time()

print("[INFO] 캘리브레이션 시작. q = 종료, r = 재캘리브레이션, c = 차량 개입 취소")

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

        # ================= 캘리브레이션 =================
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
                    baseline  = float(np.percentile(np.array(calib_samples), CALIB_PERCENTILE))
                    thr_close = baseline * CLOSE_RATIO
                    thr_open  = baseline * OPEN_RATIO
                    mode = "RUN"
                    ev_since = now
                    print(f"[CALIB] baseline  {baseline:.3f}")
                    print(f"[CALIB] close/open {thr_close:.3f} / {thr_open:.3f}")
            else:
                cv2.putText(frame, "NO FACE", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        # ================= 실행 =================
        else:
            face_ok = ear is not None
            ev_raw = "NORMAL"

            # ----- 눈 상태 (히스테리시스) + 눈감김 분류 -----
            if face_ok:
                if eye_closed:
                    if ear > thr_open:
                        eye_closed = False
                else:
                    if ear < thr_close:
                        eye_closed = True

                if 0 < dt < 1.0:
                    window.append((now, eye_closed, dt))

                if eye_closed:
                    if closed_start is None:
                        closed_start = now
                    closed_dur = now - closed_start
                    ev_raw = event_level(closed_dur)
                else:
                    if closed_start is not None:
                        kind = classify_closure(now - closed_start)
                        counts[kind] += 1
                        if kind in ("micro", "sleep", "unresp"):
                            ms_times.append(now)
                    closed_start, closed_dur = None, 0.0
            else:
                # 얼굴 미검출: 눈 상태를 알 수 없으므로 감김 타이머 초기화
                eye_closed = False
                closed_start, closed_dur = None, 0.0

            # ----- PERCLOS / 최근 마이크로슬립 -----
            while window and now - window[0][0] > PERCLOS_WINDOW:
                window.popleft()
            if window:
                total = sum(x[2] for x in window)
                closed_t = sum(x[2] for x in window if x[1])
                perclos = (closed_t / total * 100.0) if total > 0 else 0.0
                span = now - window[0][0]
            else:
                perclos, span = 0.0, 0.0

            while ms_times and now - ms_times[0] > MS_WINDOW:
                ms_times.popleft()
            ms_recent = len(ms_times)

            # ----- 이벤트 채널: 악화는 즉시, 완화는 지연, UNRESPONSIVE는 래치 -----
            prev_ev, prev_since = ev_state, ev_since
            if EV_RANK[ev_raw] > EV_RANK[ev_state]:
                ev_state, ev_since, lower_since = ev_raw, now, None
            elif EV_RANK[ev_raw] < EV_RANK[ev_state] and ev_state != "UNRESPONSIVE":
                if lower_since is None:
                    lower_since = now
                if now - lower_since >= EVENT_DWELL_SEC:
                    ev_state, ev_since, lower_since = ev_raw, now, None
            else:
                lower_since = None

            if ev_state != prev_ev:
                held = now - prev_since
                logger.log_event(prev_ev, ev_state, ear, perclos, "EYE_CLOSURE", held)
                ts = time.strftime('%H:%M:%S')
                print(f"[{ts}] {prev_ev} -> {ev_state}  "
                      f"closed {closed_dur:.2f}s  PERCLOS {perclos:.1f}%")

                if EV_RANK[ev_state] > EV_RANK[prev_ev]:
                    ev_entries[ev_state] += 1

                # 차량 반응
                if ev_state == "SLEEP" and prev_ev == "MICROSLEEP":
                    vehicle.send("ADAS_SENSITIVE_MODE", "eye closure >= 3s", ear, perclos)
                elif ev_state == "UNRESPONSIVE":
                    for cmd in ("HAZARD_LIGHTS_ON", "DECELERATE", "PULL_OVER_MRM"):
                        vehicle.send(cmd, "no response >= 13s", ear, perclos)
                elif ev_state == "NORMAL" and prev_ev == "SLEEP":
                    vehicle.send("ADAS_NORMAL_MODE", "driver recovered", ear, perclos)

            # ----- 졸음 채널 (비긴급) -----
            if not drowsy:
                if span >= PERCLOS_MIN_SPAN and perclos >= PERCLOS_ON:
                    drowsy, drowsy_cause = True, "PERCLOS"
                elif ms_recent >= MS_DROWSY_COUNT:
                    drowsy, drowsy_cause = True, "MICROSLEEP_COUNT"
                if drowsy:
                    drowsy_entries += 1
                    logger.log_event("AWAKE", "DROWSY", ear, perclos, drowsy_cause, 0.0)
                    print(f"[{time.strftime('%H:%M:%S')}] AWAKE -> DROWSY  by {drowsy_cause}  "
                          f"PERCLOS {perclos:.1f}%  MS(5m) {ms_recent}")
                    alarm.chime()
                    vehicle.send("SUGGEST_REST_AREA", drowsy_cause, ear, perclos)
            else:
                if perclos < PERCLOS_OFF and ms_recent < MS_DROWSY_COUNT:
                    drowsy = False
                    logger.log_event("DROWSY", "AWAKE", ear, perclos, "RECOVERED", 0.0)
                    print(f"[{time.strftime('%H:%M:%S')}] DROWSY -> AWAKE  "
                          f"PERCLOS {perclos:.1f}%")

            alarm.set_level(ev_state)
            logger.log_frame(fps, ear, eye_closed, closed_dur,
                             perclos, ev_state, drowsy, face_ok)

            # ----- 화면 -----
            c = EV_COLOR[ev_state]
            if ev_state in ("SLEEP", "UNRESPONSIVE"):
                cv2.rectangle(frame, (0, 0), (w - 1, h - 1), c, 12)

            if face_ok:
                cv2.putText(frame, f"EAR {ear:.3f}", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(frame, "CLOSED" if eye_closed else "OPEN",
                            (170, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 0, 255) if eye_closed else (0, 255, 0), 2)
            else:
                cv2.putText(frame, "NO FACE", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.putText(frame, ev_state, (10, 95),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.1, c, 3)
            if ev_state != "NORMAL":
                cv2.putText(frame, f"held {now - ev_since:.1f}s", (10, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1)
            cv2.putText(frame, f"closed {closed_dur:.2f}s", (10, 145),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            if face_ok:
                bar = int(min(ear, 0.4) / 0.4 * 300)
                xc = int(min(thr_close, 0.4) / 0.4 * 300)
                xo = int(min(thr_open,  0.4) / 0.4 * 300)
                cv2.rectangle(frame, (10, 155), (310, 175), (80, 80, 80), 1)
                cv2.rectangle(frame, (10, 155), (10 + bar, 175), c, -1)
                cv2.line(frame, (10 + xc, 150), (10 + xc, 180), (255, 0, 255), 2)
                cv2.line(frame, (10 + xo, 150), (10 + xo, 180), (255, 255, 0), 2)

            # PERCLOS 막대 (0~30% 표시)
            pc = (0, 165, 255) if drowsy else (0, 255, 0)
            pbar = int(min(perclos, 30.0) / 30.0 * 300)
            cv2.rectangle(frame, (10, 195), (310, 215), (80, 80, 80), 1)
            cv2.rectangle(frame, (10, 195), (10 + pbar, 215), pc, -1)
            for mk, mc in [(PERCLOS_OFF, (200, 200, 200)), (PERCLOS_ON, (0, 165, 255))]:
                mx = int(mk / 30.0 * 300)
                cv2.line(frame, (10 + mx, 190), (10 + mx, 220), mc, 1)
            cv2.putText(frame, f"PERCLOS {perclos:.1f}% ({span:.0f}s)  MS(5m) {ms_recent}",
                        (10, 238), cv2.FONT_HERSHEY_SIMPLEX, 0.55, pc, 2)
            cv2.putText(frame,
                        f"blink {counts['blink']}  long {counts['long']}  "
                        f"micro {counts['micro']}  sleep {counts['sleep']}",
                        (10, 262), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            if ev_state == "UNRESPONSIVE":
                cv2.putText(frame, "VEHICLE: PULL OVER  |  press C = driver override",
                            (10, h - 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2)

            if drowsy:
                cv2.rectangle(frame, (0, h - 40), (w, h), (0, 140, 255), -1)
                cv2.putText(frame, "DROWSY - TAKE A BREAK", (10, h - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

        # ----- 공통 표시 -----
        prev = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
        cv2.putText(frame, f"FPS {fps:.1f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        el = time.time() - logger.t0
        cv2.putText(frame, f"{int(el // 60):02d}:{int(el % 60):02d}",
                    (w - 90, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        cv2.imshow("Drowsiness Detection v4 (Euro NCAP)", frame)

        if AUTO_STOP_SEC > 0 and time.time() - logger.t0 >= AUTO_STOP_SEC:
            print(f"[INFO] {AUTO_STOP_SEC}초 경과, 자동 종료")
            break

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        elif key == ord('c') and mode == "RUN" and ev_state == "UNRESPONSIVE":
            logger.log_event("UNRESPONSIVE", "NORMAL", ear, perclos,
                             "DRIVER_OVERRIDE", now - ev_since)
            vehicle.send("INTERVENTION_CANCELLED", "driver override (C key)", ear, perclos)
            ev_state, ev_since, lower_since = "NORMAL", now, None
            eye_closed = False
            closed_start, closed_dur = None, 0.0
            alarm.set_level("NORMAL")

        elif key == ord('r'):
            logger.log_event(ev_state, "RECALIB", None, perclos, "MANUAL", 0.0)
            alarm.set_level("NORMAL")
            mode, calib_samples, calib_start = "CALIB", [], None
            window.clear()
            ms_times.clear()
            eye_closed = False
            closed_start, closed_dur = None, 0.0
            ev_state, ev_since, lower_since = "NORMAL", time.time(), None
            drowsy, drowsy_cause = False, "-"
            for k in counts:
                counts[k] = 0
            for k in ev_entries:
                ev_entries[k] = 0
            drowsy_entries = 0
            print("[INFO] 재캘리브레이션")

alarm.stop()
cap.release()
cv2.destroyAllWindows()
logger.close()

# ===== 세션 요약 =====
dur = time.time() - logger.t0
print(f"\n{'=' * 52}")
print(f"세션 요약  {SESSION_TAG or '(태그없음)'}")
print(f"{'=' * 52}")
print(f"총 시간            {dur:.1f}s")
print(f"평균 FPS           {logger.frame_n / dur:.1f}" if dur > 0 else "평균 FPS           -")
if baseline is not None:
    print(f"baseline           {baseline:.3f}")
    print(f"임계 close/open    {thr_close:.3f} / {thr_open:.3f}")
print("-" * 52)
print("[눈감김 분류]")
print(f"  정상 깜빡임 (<=0.4s)   {counts['blink']}회")
print(f"  긴 깜빡임 (0.4~1s)     {counts['long']}회")
print(f"  마이크로슬립 (1~3s)    {counts['micro']}회")
print(f"  수면 (3~13s)           {counts['sleep']}회")
print(f"  무반응 (>=13s)         {counts['unresp']}회")
print("-" * 52)
print("[이벤트 채널 경보 진입]")
print(f"  MICROSLEEP             {ev_entries['MICROSLEEP']}회")
print(f"  SLEEP                  {ev_entries['SLEEP']}회")
print(f"  UNRESPONSIVE           {ev_entries['UNRESPONSIVE']}회")
print("-" * 52)
print("[졸음 채널]")
print(f"  DROWSY 진입            {drowsy_entries}회")
print(f"  최종 PERCLOS           {perclos:.1f}%")
print("-" * 52)
print(f"[차량 명령]            {len(vehicle.history)}건")
for t, cmd, reason in vehicle.history:
    print(f"  {t - logger.t0:7.1f}s  {cmd:<24} {reason}")
print(f"{'=' * 52}")