"""
pi_sim_sender.py  -  라즈베리파이 흉내 (노트북 -> 차량용 ESP32-A, USB 시리얼)

라즈베리파이 ROS2 노드가 할 일을 노트북에서 대신 한다.
초당 10회 "S,<위험코드>,<속도>" 를 보내고, ESP32-A의 상태 보고("A,...")를 출력한다.

실행
  python pi_sim_sender.py --port COM5

키 (이 창을 클릭한 상태에서)
  0 정상   1 급제동   2 비상정차   3 상태모름
  + / -   속도 0.05 m/s 올리기/내리기
  p       전송 일시정지/재개 (라즈베리파이가 죽은 상황 흉내)
  q       종료

주의: Arduino IDE 시리얼 모니터가 같은 포트를 열고 있으면 실패한다. 먼저 닫을 것.
"""

import argparse
import sys
import threading
import time

try:
    import serial
except ImportError:
    raise SystemExit("[ERROR] pyserial이 없습니다.  pip install pyserial")

try:
    import msvcrt          # Windows 키 입력
except ImportError:
    msvcrt = None

CODE_NAME = {0: "정상", 1: "급제동", 2: "비상정차", 3: "상태모름"}

parser = argparse.ArgumentParser()
parser.add_argument("--port", required=True, help="ESP32-A 포트 (예: COM5, /dev/ttyUSB0)")
parser.add_argument("--baud", type=int, default=115200)
parser.add_argument("--hz", type=float, default=10.0)
args = parser.parse_args()

ser = serial.Serial()
ser.port = args.port
ser.baudrate = args.baud
ser.timeout = 0.1
ser.dtr = False            # 포트 열 때 ESP32가 재부팅되는 것 줄이기
ser.rts = False
try:
    ser.open()
except serial.SerialException as e:
    raise SystemExit(f"[ERROR] {args.port} 를 열 수 없습니다: {e}\n"
                     f"  - 포트 번호 확인 (장치 관리자 > 포트)\n"
                     f"  - Arduino IDE 시리얼 모니터 닫기")

state = {"code": 0, "speed": 0.20, "paused": False, "running": True, "sent": 0}
lock = threading.Lock()


def reader():
    """ESP32-A가 보내는 줄을 읽어서 출력."""
    while state["running"]:
        try:
            raw = ser.readline()
        except serial.SerialException:
            break
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if line.startswith("A,"):
            f = line.split(",")
            if len(f) == 7:
                link = "연결" if f[1] == "1" else "끊김"
                code = int(f[2]) if f[2].isdigit() else -1
                print(f"  [ESP32-A] 라즈베리파이 {link} | 방송 코드 {f[2]}({CODE_NAME.get(code, '?')}) "
                      f"속도 {f[3]} | 받음 {f[4]} 오류 {f[5]} 방송실패 {f[6]}")
                continue
        if line:
            print(f"  [ESP32-A] {line}")


def keys():
    """키 입력으로 상태 바꾸기."""
    def handle(k):
        with lock:
            if k in "0123":
                state["code"] = int(k)
                print(f">> 보내는 코드: {k} ({CODE_NAME[int(k)]})")
            elif k == "+":
                state["speed"] = round(min(state["speed"] + 0.05, 2.0), 2)
                print(f">> 속도 {state['speed']:.2f} m/s")
            elif k == "-":
                state["speed"] = round(max(state["speed"] - 0.05, 0.0), 2)
                print(f">> 속도 {state['speed']:.2f} m/s")
            elif k == "p":
                state["paused"] = not state["paused"]
                print(">> 전송 일시정지 (라즈베리파이 죽음 흉내)" if state["paused"] else ">> 전송 재개")
            elif k == "q":
                state["running"] = False

    if msvcrt:
        while state["running"]:
            if msvcrt.kbhit():
                handle(msvcrt.getwch().lower())
            time.sleep(0.02)
    else:                               # Windows가 아니면 한 줄씩 입력
        while state["running"]:
            try:
                for k in input().strip().lower():
                    handle(k)
            except EOFError:
                break


print(f"[SIM] {args.port} @ {args.baud}bps, 초당 {args.hz:g}회 전송")
print("[SIM] 키: 0~3 코드 / + - 속도 / p 일시정지 / q 종료")
threading.Thread(target=reader, daemon=True).start()
threading.Thread(target=keys, daemon=True).start()

period = 1.0 / args.hz
next_t = time.time()
try:
    while state["running"]:
        with lock:
            paused = state["paused"]
            line = f"S,{state['code']},{state['speed']:.2f}\n"
        if not paused:
            ser.write(line.encode("ascii"))
            state["sent"] += 1
        next_t += period
        time.sleep(max(0.0, next_t - time.time()))
except KeyboardInterrupt:
    pass
finally:
    state["running"] = False
    time.sleep(0.2)
    ser.close()
    print(f"\n[SIM] 종료. 보낸 줄 {state['sent']}개")
