"""
drowsy_v5.py  -  라즈베리파이 연동 버전

v4 대비 변경점
1. 가짜 차량 명령(VehicleInterface) 제거
2. 라즈베리파이로 운전자 상태를 UDP 전송 (PiLink)
   - 초당 10번 계속 전송 (하트비트). 프로그램이 멈추면 전송도 멈춤
     -> 라즈베리파이가 "1초간 안 오면 끊김"으로 판단할 수 있음
   - 메시지: {"seq": 1523, "anomaly": true, "state": "SLEEP", "closed_dur": 3.24}
   - 라즈베리파이는 anomaly 값만 /sensors/bio_anomaly (std_msgs/Bool) 로 발행
3. bio_anomaly 래치
   - 눈감김이 SLEEP(3초) 이상이면 anomaly = True
   - 눈을 떠도 True 유지 (한 프레임만 False로 튀어도 차가 갓길에서 재출발하므로)
   - C 키(운전자 조작)로만 해제
   - 얼굴이 사라진 시간도 눈감김으로 센다 (쓰러짐·고개 떨굼)
   - Pi의 decision_maker도 따로 래치한다(bio_latch). 그래서 C 키로 해제해도
     **차는 Pi 노드를 재시작하기 전까지 정차 상태를 유지한다** (UN R157: 재출발 금지)

실행
  python drowsy_v5.py                  # 라즈베리파이(raspberrypi.local)로 전송
  python drowsy_v5.py --pi 127.0.0.1   # 내 노트북으로 전송 (수신 테스트용)

키: q = 종료, r = 재캘리브레이션, c = bio_anomaly 해제(운전자 조작)
"""

import argparse
import csv
import json
import os
import socket
import threading
import time
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

# --- 라즈베리파이 연동 ---
PI_IP_DEFAULT   = "raspberrypi.local"   # IP는 DHCP라 바뀐다. mDNS 이름을 쓴다
PI_PORT         = 5005
SEND_HZ         = 10          # 초당 전송 횟수
ANOMALY_TRIGGER = "SLEEP"     # 이 단계 이상이면 bio_anomaly = True (래치)

LEFT_EYE  = [33, 160, 158, 133, 153, 144]
RIGHT_EYE = [362, 385, 387, 263, 373, 380]

# --- 캘리브레이션 / 눈 상태 ---
CALIB_SECONDS    = 8.0
CALIB_PERCENTILE = 80
CLOSE_RATIO      = 0.60     # 감김 진입 (baseline 대비)
OPEN_RATIO       = 0.70     # 뜸 복귀 (히스테리시스)

# --- 이벤트 채널 (Euro NCAP 기준) ---
BLINK_MAX        = 0.40     # 이하 = 정상 깜빡임
MICROSLEEP_SEC   = 1.0      # 마이크로슬립 (Euro NCAP: 1~2초)
SLEEP_SEC        = 3.0      # 수면 -> 차량 개입 요청 (Euro NCAP: 연속 눈감김 >=3초 = asleep)
# 무반응: Euro NCAP 실제 기준은 6초(눈감김/시선이탈 지속) 또는 경고 후 3초 내 미복귀.
# 이전엔 13.0으로 되어 있었는데 근거 없는 값이었다 — 규격값(6.0)으로 맞춘다.
# 어차피 SLEEP(3초)에서 이미 bio_anomaly=True가 확정되므로 Pi로 가는 신호엔 영향 없고,
# 화면 표시 상태 이름(UNRESPONSIVE)이 언제 뜨는지만 바뀐다.
UNRESPONSIVE_SEC = 6.0      # 무반응 (Euro NCAP: 눈감김/시선이탈 지속 >=6초)
EVENT_DWELL_SEC  = 1.5      # 눈 뜬 뒤 경고 최소 유지

# --- 졸음 채널 (누적 지표, 비긴급) ---
PERCLOS_WINDOW   = 60.0
PERCLOS_MIN_SPAN = 30.0
PERCLOS_ON       = 15.0
PERCLOS_OFF      = 12.0
MS_WINDOW        = 300.0
MS_DROWSY_COUNT  = 2

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


# ===== 라즈베리파이 전송 =====
class PiLink:
    """
    운전자 상태를 UDP로 라즈베리파이에 보낸다.
    메인 루프에서 매 프레임 호출한다 -> 메인 루프가 멈추면 전송도 멈추므로
    라즈베리파이 쪽에서 '수신 끊김 = 노트북 이상'으로 감지할 수 있다(하트비트).
    """

    def __init__(self, host, port, hz):
        # 호스트명(raspberrypi.local)은 시작할 때 한 번만 IP로 바꾼다.
        # 매 전송마다 이름을 풀면 mDNS 조회가 루프를 느리게 만든다.
        try:
            ip = socket.gethostbyname(host)
        except OSError as e:
            raise SystemExit(f"[ERROR] {host} 를 찾을 수 없습니다 ({e}). "
                             f"Pi와 같은 Wi-Fi인지 확인하거나 --pi 로 IP를 직접 주세요.")
        self.addr = (ip, port)
        self.period = 1.0 / hz
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.last_send = 0.0
        self.errors = 0
        self.last_error = ""

    def maybe_send(self, now, anomaly, state, closed_dur, force=False):
        """주기가 됐거나 force=True면 전송."""
        if not force and now - self.last_send < self.period:
            return
        self.seq += 1
        msg = {
            "seq": self.seq,
            "anomaly": bool(anomaly),
            "state": state,
            "closed_dur": round(float(closed_dur), 2),
        }
        try:
            self.sock.sendto(json.dumps(msg).encode("utf-8"), self.addr)
        except OSError as e:
            self.errors += 1
            self.last_error = str(e)
        self.last_send = now

    def close(self):
        self.sock.close()


# ===== 로깅 =====
class Logger:
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
            "perclos", "event_state", "drowsy", "anomaly", "face_detected",
        ])
        self.ew.writerow([
            "elapsed_s", "timestamp", "from_state", "to_state",
            "ear", "perclos", "cause", "held_s",
        ])

        self.t0 = time.time()
        self.frame_n = 0
        self.event_n = 0

    def log_frame(self, fps, ear, eye_closed, closed_dur,
                  perclos, ev_state, drowsy, anomaly, face_ok):
        self.fw.writerow([
            f"{time.time() - self.t0:.3f}",
            f"{fps:.1f}",
            f"{ear:.4f}" if ear is not None else "",
            int(eye_closed),
            f"{closed_dur:.3f}",
            f"{perclos:.2f}",
            ev_state,
            int(drowsy),
            int(anomaly),
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
    if closed_dur >= UNRESPONSIVE_SEC:
        return "UNRESPONSIVE"
    if closed_dur >= SLEEP_SEC:
        return "SLEEP"
    if closed_dur >= MICROSLEEP_SEC:
        return "MICROSLEEP"
    return "NORMAL"


def classify_closure(d):
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
parser = argparse.ArgumentParser()
parser.add_argument("--pi", default=PI_IP_DEFAULT,
                    help="라즈베리파이 호스트명 또는 IP (테스트 시 127.0.0.1)")
args = parser.parse_args()

SESSION_TAG = input("테스트 조건 태그 (예: pi_link_test / 엔터=생략): ").strip()
logger = Logger(SESSION_TAG)
alarm  = Alarm()
link   = PiLink(args.pi, PI_PORT, SEND_HZ)
print(f"[PI] UDP 전송 대상 {args.pi}:{PI_PORT}  (초당 {SEND_HZ}회)")

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
window = deque()
perclos, span = 0.0, 0.0
ms_times = deque()
ms_recent = 0

# --- bio_anomaly (라즈베리파이로 보내는 값) ---
anomaly = False
anomaly_since = None
anomaly_count = 0
release_count = 0

# --- 통계 ---
counts = {"blink": 0, "long": 0, "micro": 0, "sleep": 0, "unresp": 0}
ev_entries = {"MICROSLEEP": 0, "SLEEP": 0, "UNRESPONSIVE": 0}
drowsy_entries = 0

prev, fps = time.time(), 0.0
ear = None
now = time.time()

print("[INFO] 캘리브레이션 시작. q = 종료, r = 재캘리브레이션, c = bio_anomaly 해제")

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
                # 얼굴 소실도 눈감김과 똑같이 누적한다. 운전자가 쓰러지거나 고개를 떨구면
                # 얼굴이 화면에서 사라지는데, 여기서 리셋하면 가장 위급한 상황이 NORMAL로 남는다.
                # (정면을 3초 넘게 벗어나도 SLEEP으로 잡힌다 — 시연 중 옆을 오래 보지 말 것)
                if closed_start is None:
                    closed_start = now
                closed_dur = now - closed_start
                ev_raw = event_level(closed_dur)

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
                print(f"[{time.strftime('%H:%M:%S')}] {prev_ev} -> {ev_state}  "
                      f"closed {closed_dur:.2f}s  PERCLOS {perclos:.1f}%")
                if EV_RANK[ev_state] > EV_RANK[prev_ev]:
                    ev_entries[ev_state] += 1

            # ----- bio_anomaly 래치: SLEEP 이상이면 True, C 키로만 해제 -----
            if not anomaly and EV_RANK[ev_state] >= EV_RANK[ANOMALY_TRIGGER]:
                anomaly = True
                anomaly_since = now
                anomaly_count += 1
                logger.log_event("PI_LINK", "ANOMALY_TRUE", ear, perclos,
                                 f"{ev_state} (closed {closed_dur:.2f}s)", 0.0)
                print(f"[{time.strftime('%H:%M:%S')}] [PI] bio_anomaly = True  "
                      f"({ev_state}, closed {closed_dur:.2f}s)")
                link.maybe_send(now, anomaly, ev_state, closed_dur, force=True)

            # ----- 졸음 채널 (비긴급, 로컬 안내만) -----
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
            else:
                if perclos < PERCLOS_OFF and ms_recent < MS_DROWSY_COUNT:
                    drowsy = False
                    logger.log_event("DROWSY", "AWAKE", ear, perclos, "RECOVERED", 0.0)
                    print(f"[{time.strftime('%H:%M:%S')}] DROWSY -> AWAKE  "
                          f"PERCLOS {perclos:.1f}%")

            alarm.set_level(ev_state)
            logger.log_frame(fps, ear, eye_closed, closed_dur,
                             perclos, ev_state, drowsy, anomaly, face_ok)

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

            if anomaly:
                cv2.rectangle(frame, (0, h - 82), (w, h - 44), (0, 0, 200), -1)
                cv2.putText(frame, "MRM REQUESTED  bio_anomaly=True   [C] release",
                            (10, h - 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            if drowsy:
                cv2.rectangle(frame, (0, h - 40), (w, h), (0, 140, 255), -1)
                cv2.putText(frame, "DROWSY - TAKE A BREAK", (10, h - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

        # ----- 라즈베리파이 전송 (캘리브레이션 중에도 하트비트) -----
        link.maybe_send(now, anomaly, ev_state if mode == "RUN" else "CALIB", closed_dur)

        # ----- 공통 표시 -----
        prev = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
        cv2.putText(frame, f"FPS {fps:.1f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        el = time.time() - logger.t0
        cv2.putText(frame, f"{int(el // 60):02d}:{int(el % 60):02d}",
                    (w - 90, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        link_color = (0, 0, 255) if link.errors else (180, 180, 180)
        cv2.putText(frame, f"-> PI {link.addr[0]}:{link.addr[1]}  seq {link.seq}  err {link.errors}",
                    (10, 285), cv2.FONT_HERSHEY_SIMPLEX, 0.45, link_color, 1)

        cv2.imshow("Drowsiness Detection v5 (Pi link)", frame)

        if AUTO_STOP_SEC > 0 and time.time() - logger.t0 >= AUTO_STOP_SEC:
            print(f"[INFO] {AUTO_STOP_SEC}초 경과, 자동 종료")
            break

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        elif key == ord('c') and anomaly:
            # 운전자 조작으로 해제
            held = now - anomaly_since if anomaly_since else 0.0
            anomaly = False
            release_count += 1
            logger.log_event("PI_LINK", "ANOMALY_FALSE", ear, perclos, "DRIVER_OVERRIDE", held)
            print(f"[{time.strftime('%H:%M:%S')}] [PI] bio_anomaly = False  "
                  f"(C 키 해제, {held:.1f}초 유지됨)")
            ev_state, ev_since, lower_since = "NORMAL", now, None
            eye_closed = False
            closed_start, closed_dur = None, 0.0
            alarm.set_level("NORMAL")
            link.maybe_send(now, anomaly, ev_state, closed_dur, force=True)

        elif key == ord('r'):
            # 재캘리브레이션 (bio_anomaly는 C 키로만 해제되므로 건드리지 않음)
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
link.close()
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
print("[라즈베리파이 전송]")
print(f"  대상                   {link.addr[0]}:{link.addr[1]}")
print(f"  전송 패킷              {link.seq}개")
print(f"  전송 오류              {link.errors}개" +
      (f"  (마지막: {link.last_error})" if link.errors else ""))
print(f"  bio_anomaly True       {anomaly_count}회")
print(f"  C 키 해제              {release_count}회")
print(f"  종료 시 anomaly        {anomaly}")
print(f"{'=' * 52}")
