"""
udp_test_receiver.py  -  UDP 수신 테스트 (ROS 없이, 파이썬 기본 기능만 사용)

drowsy_v5.py 가 보내는 메시지를 받아서 화면에 출력한다.
노트북에서도, 라즈베리파이에서도 그대로 실행된다.

  python udp_test_receiver.py          # 기본 포트 5005
  python udp_test_receiver.py 5005

라즈베리파이 ROS2 브릿지 노드를 짤 때 이 파일의 수신/끊김 감지 로직을 참고하면 된다.
"""

import json
import socket
import sys
import time

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 5005
TIMEOUT_SEC = 1.0          # 이 시간 동안 못 받으면 끊김

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PORT))
sock.settimeout(0.2)       # 0.2초마다 깨어나서 끊김 여부 확인

print(f"[RX] UDP {PORT} 포트에서 대기 중... (Ctrl+C 종료)")

count = 0
lost = 0
last_seq = None
last_rx = None
last_anomaly = None
last_state = None
link_ok = False
last_status = 0.0

try:
    while True:
        try:
            data, addr = sock.recvfrom(1024)
        except socket.timeout:
            data = None

        now = time.time()

        if data is not None:
            try:
                msg = json.loads(data.decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                print(f"[RX] 해석할 수 없는 데이터: {data[:60]!r}")
                continue

            count += 1
            seq = msg.get("seq")
            if isinstance(seq, int) and isinstance(last_seq, int) and seq > last_seq + 1:
                lost += seq - last_seq - 1
            last_seq = seq
            last_rx = now

            if not link_ok:
                print(f"[RX] 연결됨 <- {addr[0]}")
                link_ok = True

            anomaly = bool(msg.get("anomaly", False))
            last_state = msg.get("state")
            if anomaly != last_anomaly:
                print(f"[RX] ***** anomaly = {anomaly} *****  "
                      f"state={last_state}  closed={msg.get('closed_dur')}s")
                last_anomaly = anomaly

            if now - last_status >= 1.0:
                print(f"[RX] 수신 {count}개  seq {seq}  손실 {lost}  "
                      f"anomaly={anomaly}  state={last_state}")
                last_status = now

        # 끊김 감지
        if link_ok and last_rx is not None and now - last_rx > TIMEOUT_SEC:
            print(f"[RX] !!! {TIMEOUT_SEC:.0f}초 이상 수신 없음 -> 연결 끊김 "
                  f"(마지막 anomaly={last_anomaly})")
            link_ok = False

except KeyboardInterrupt:
    print(f"\n[RX] 종료. 총 {count}개 수신, 손실 추정 {lost}개")
finally:
    sock.close()
