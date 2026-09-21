# 운전자 이상신호 인터페이스 (노트북 웹캠 ↔ Pi)

담당: 정수영 · 노트북 코드: [`driver_monitor/drowsy_v5.py`](../driver_monitor/drowsy_v5.py) · Pi 수신: `safecar/safecar/comms/sensor_bridge_node.py`

> 📐 **감지 알고리즘 스펙은 [`docs/DRIVER_MONITORING_SPEC.md`](DRIVER_MONITORING_SPEC.md)** 에 따로 있다.
> 이 문서는 **노트북과 Pi의 접점(UDP 메시지 규약)** 만 다룬다.

## 작업 경계

운전자 이상 감지(웹캠)는 **윈도우 노트북에서 파이썬으로 직접 돈다 (ROS·VM 없음).**
Pi와의 접점은 **UDP 5005 포트 하나뿐**이다.

```
[노트북 · 윈도우]                                  [Pi · safecar]
 웹캠 → drowsy_v5.py ──UDP JSON, 10Hz──▶ sensor_bridge_node ──/sensors/bio_anomaly──▶ decision_maker → MRM 갓길 정차
        (mediapipe)       port 5005        (bio_source:=udp)      (std_msgs/Bool)
```

왜 ROS가 아니라 UDP인가: 기계 경계를 넘는 건 불리언 하나(2Hz 이상)뿐이라 VM·ROS 2·DDS를 얹을 이유가 없다.
최신 값만 의미 있는 주기적 상태 신호라, 빠진 패킷을 재전송하느라 뒤 패킷을 붙잡는 TCP보다 UDP가 맞다.

## 메시지 규약 (이것만 지키면 됨)

| 항목 | 값 |
|---|---|
| 전송 | UDP, 노트북 → Pi `raspberrypi.local:5005` |
| 형식 | UTF-8 JSON 한 줄: `{"seq": 1523, "anomaly": true, "state": "SLEEP", "closed_dur": 3.24}` |
| 주기 | **초당 10회 계속** (이상이 없어도 보낸다 = 하트비트) |
| 필수 필드 | `anomaly`(bool). 나머지는 로그·손실 집계용 |

의미 규약:

- `anomaly=false` — 운전자 정상. **정상일 때도 계속 보내야** Pi가 연결이 살아 있음을 안다.
- `anomaly=true` — 운전자 무반응. Pi가 즉시 `MRM_PULL_OVER`로 전환해 갓길 정차를 시작한다.
- **latch는 노트북 책임** — 눈감김(또는 얼굴 소실)이 3초(`SLEEP`) 이상이면 `true`로 올리고,
  운전자가 C 키로 해제할 때까지 유지한다. 한 프레임이라도 `true→false`로 튀면 차가 갓길에서 재출발하기 때문이다.
- Pi의 `decision_maker`도 따로 래치한다(`bio_latch`, UN R157 재출발 금지). 그래서 **노트북에서 해제해도
  차는 Pi 노드를 재시작하기 전까지 정차 상태를 유지한다.** 시연 중 C 키를 눌러도 차가 다시 가지 않는 게 정상이다.

## 끊김 처리 (Pi 쪽)

`udp_timeout_sec`(기본 1초) 동안 패킷이 없으면 끊김으로 본다.

- 마지막 값이 `true`였으면 **`true` 유지** — 갓길 정차 도중 재출발 방지
- 마지막 값이 `false`였으면 **`false` 유지 + 경고 로그** — 운전자 감시 없이 주행은 계속된다.
  (Wi-Fi가 잠깐 흔들려도 차가 갓길에 서지 않게 한 시연 안정성 쪽 선택. 실차라면 끊김 = MRM이 맞다)

## 각자 독립 테스트

**노트북 쪽 (Pi 없이)** — 내 노트북으로 보내서 규약대로 나오는지 확인:

```bash
python driver_monitor/udp_test_receiver.py          # 터미널 1
python driver_monitor/drowsy_v5.py --pi 127.0.0.1   # 터미널 2
```

**Pi 쪽 (노트북 없이)** — 손으로 패킷을 쏴서 MRM 동작 검증:

```bash
ros2 launch safecar safecar.launch.py            # bio_source 기본값 udp
# 다른 터미널
python3 -c "import socket; socket.socket(2, 2).sendto(b'{\"seq\":1,\"anomaly\":true}', ('127.0.0.1', 5005))"
ros2 topic echo /control/driving_state           # NORMAL → MRM_PULL_OVER 전환 확인
```

노트북 없이 시간 기반으로 MRM만 보고 싶으면 시뮬레이션을 쓴다:

```bash
ros2 launch safecar safecar.launch.py lane_follow:=true bio_source:=sim anomaly_delay_sec:=10.0
```

## 통합 시연 실행 순서

```bash
# 1) Pi
~/safecar_start.sh        # 통합 launch (bio_source=udp)
~/safecar_drive.sh        # 주행 시작
# 2) 노트북 (Pi와 같은 Wi-Fi/핫스팟)
python driver_monitor/drowsy_v5.py
```

노트북 화면 하단의 `-> PI <IP>:5005 seq ...`가 올라가고 `err 0`이면 전송 중이다.
Pi 로그에 `노트북 연결됨 <- <노트북 IP>`가 뜨면 연결 완료.

## 시연 영상 촬영 요구사항

차체가 작아 사람이 탑승할 수 없으므로, 운전자(노트북 앞)와 차량(트랙)은 **따로 촬영해 편집으로 붙인다.**
그래서 "운전자 이상 → 차량 반응"의 인과를 화면으로 증명할 수 있어야 한다.

- 노트북 창: 웹캠 영상 + EAR·상태(`MICROSLEEP` / `SLEEP`) + `MRM REQUESTED bio_anomaly=True` 배너 (구현됨)
- Pi 쪽 상태: `ros2 topic echo /control/driving_state` 출력(`NORMAL` → `MRM_PULL_OVER`)을 같은 화면에 띄운다
- 차량 주행 영상은 폰으로 따로 찍어 편집 시 붙인다.
