# V2V 위험 경고 인터페이스 (Pi ↔ 차량 ESP32 ↔ 뒤차 화면)

담당: 정수영 · Pi 노드: `safecar/safecar/comms/v2x_bridge_node.py`
· 펌웨어: [`telemetry/esp32_vehicle`](../telemetry/esp32_vehicle/esp32_vehicle.ino), [`telemetry/esp32_display`](../telemetry/esp32_display/esp32_display.ino)

> 운전자 이상신호(노트북 → Pi)는 [`DRIVER_SIGNAL_CONTRACT.md`](DRIVER_SIGNAL_CONTRACT.md)에 따로 있다.
> 이 문서는 **우리 차의 위험 상태를 뒤차에 알리는 경로**만 다룬다.

## 작업 경계

```
[Pi · safecar]                          [ESP32-A · 차량]              [ESP32-B · 뒤차 화면]
 /control/driving_state ─┐               시리얼 → BSM                  BSM → 넥션 페이지
 /odom (속도)  ───────────┼▶ v2x_bridge_node ──USB 115200──▶ ──ESP-NOW 방송──▶  ──UART 9600──▶ 넥션
                          │   "S,<code>,<speed>" 10Hz        10Hz
 /v2x/status  ◀──────────┘◀── "A,..." 1Hz 상태 보고
```

한 방향이다. 뒤차 화면은 우리 차에 아무것도 보내지 않는다.

## 위험코드

| 코드 | `/control/driving_state` | 뒤차 넥션 | 화면 |
|---|---|---|---|
| 0 | `NORMAL` | page 0 | 이상없음 |
| 1 | `EMERGENCY_BRAKE` | page 1 | 전방차량 급제동 |
| 2 | `MRM_PULL_OVER` | page 2 | 전방차량 비상정차 (운전자 이상) |
| 3 | 위 셋이 아닌 값, 또는 신호 끊김 | page 3 | 전방차량 신호없음 |

상태 이름은 `safecar/safecar/protocol.py`에서 불러온다. 거기에 새 상태(예: `AVOID`)가 생기면
`v2x_bridge_node.py`의 `STATE_TO_CODE`에 추가하기 전까지는 **코드 3으로 방송된다** (모르는 상태를 정상으로 보여주지 않기 위해).

## 시리얼 규약 (Pi → ESP32-A)

| 항목 | 값 |
|---|---|
| 연결 | USB, 115200bps, 장치 이름 `/dev/esp32_v2x` (udev, 아래 참고) |
| 형식 | ASCII 한 줄 `S,<code>,<speed_mps>\n` 예) `S,2,0.15` |
| 주기 | **초당 10회 계속** (상태가 안 바뀌어도 = 하트비트) |
| 속도 | `/odom`의 `twist.twist.linear.x` (m/s). `/odom`이 1초 넘게 없으면 0 |

ESP32-A는 형식이 조금이라도 다르면 그 줄을 버린다 (코드 0~3 밖, 속도 ±10 m/s 밖, 필드 수 불일치, 63자 초과).

### 상태 보고 (ESP32-A → Pi, 1Hz)

```
A,<link>,<code>,<speed>,<rx_count>,<bad_count>,<send_fail>
```

`link`=1이면 Pi 신호를 받는 중. `v2x_bridge_node`가 이 줄을 `/v2x/status`(String)로 그대로 발행한다.
`#`로 시작하는 줄은 사람이 읽는 안내문이고, 그 외 줄(부팅 메시지 등)은 무시한다.

## 무선 규약 (ESP32-A → ESP32-B, ESP-NOW 방송)

```c
typedef struct {
  uint32_t vehicle_id;    // 우리 차 = 1
  uint32_t timestamp;     // ESP32-A 부팅 후 ms
  float    speed;         // m/s
  uint8_t  danger_code;   // 위 표
} __attribute__((packed)) BSM;   // 13바이트
```

- 목적지 `FF:FF:FF:FF:FF:FF` 방송, 초당 10회
- ESP32-B는 **크기 13바이트 · `vehicle_id`=1 · 코드 0~3** 인 것만 반영하고 나머지는 무시한다

## 끊김 처리 — 어디가 멈춰도 뒤차 화면은 "신호없음"

| 멈춘 곳 | 누가 알아채나 | 결과 |
|---|---|---|
| `decision_maker` (상태 1초 안 옴) | `v2x_bridge_node` | 코드 3 방송 |
| `v2x_bridge_node` / Pi 전체 (시리얼 1초 안 옴) | ESP32-A 펌웨어 | 코드 3 방송 |
| ESP32-A (무선 1초 안 옴) | ESP32-B 펌웨어 | page 3 |

"통신이 끊겼는데 화면은 계속 이상없음"인 상황을 만들지 않는 게 목적이다.
ESP32-B는 켜자마자도 page 3에서 시작한다 (아직 아무것도 못 받았으므로).

## 하드웨어

| 보드 | MAC 끝 | 역할 | 연결 |
|---|---|---|---|
| ESP32-A | `...:4C` | 차량용 | Pi USB |
| ESP32-B | `...:78` | 뒤차 화면용 | 넥션 + USB 전원(충전기/보조배터리) |

넥션 `NX3224T024_011` (2.4", 320×240 가로) ↔ ESP32-B: `+5V`–`5V`, `GND`–`GND`, 넥션 `TX`–`GPIO16`, 넥션 `RX`–`GPIO17`, 9600bps.
넥션은 `page N` 명령만 받는다 (글자는 배경 그림에 포함, 폰트·이벤트 코드 없음).

## Pi 설정 (최초 1회)

### 1. pyserial

```bash
sudo apt install python3-serial
```

### 2. 장치 이름 고정 (udev) — 라이다와 충돌 방지

YDLIDAR X4의 USB 변환 칩과 ESP32-A가 **둘 다 CP210x**(`10c4:ea60`)라서, YDLIDAR 기본 규칙
(`ydlidar_ros/startup/initenv.sh`, "CP210x면 전부 `/dev/ydlidar`")이 ESP32를 라이다로 잡을 수 있다.
`stella.rules`의 모터드라이버·AHRS처럼 **꽂는 USB 구멍 위치(`KERNELS`)** 로 둘을 나눈다.

```bash
# 라이다만 꽂은 상태에서 라이다의 구멍 위치 확인 (예: 1-1, 3-1 ...)
udevadm info -a -n /dev/ttyUSB0 | grep -m1 -E 'KERNELS=="[0-9]+-[0-9.]+"'

# ESP32-A를 다른 구멍에 꽂고 같은 방법으로 위치 확인
udevadm info -a -n /dev/ttyUSB1 | grep -m1 -E 'KERNELS=="[0-9]+-[0-9.]+"'
```

`/etc/udev/rules.d/ydlidar.rules` 의 CP210x 줄에 라이다 위치를 추가하고,
ESP32용 규칙을 새로 만든다 (`<...>`는 위에서 확인한 값):

```bash
# ydlidar.rules (수정)
KERNEL=="ttyUSB*", KERNELS=="<라이다 위치>", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"

# /etc/udev/rules.d/99-esp32-v2x.rules (새로)
KERNEL=="ttyUSB*", KERNELS=="<ESP32 위치>", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="esp32_v2x"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/ydlidar /dev/esp32_v2x     # 서로 다른 ttyUSB를 가리켜야 함
```

이후로는 **라이다와 ESP32-A를 항상 같은 구멍에** 꽂는다.

## 각자 독립 테스트

**ESP32 쪽 (Pi 없이)** — 노트북이 Pi 흉내를 낸다:

```bash
python telemetry/dev/pi_sim_sender.py --port COM8     # 키 0~3 코드, p 일시정지(Pi 죽음), q 종료
```

**Pi 쪽 (ESP32 없이)** — 노드는 ESP32가 없어도 죽지 않고 재연결만 시도한다:

```bash
ros2 run safecar v2x_bridge_node          # "ESP32-A를 열 수 없음 ... 재시도" 경고만 나오면 정상
```

## 통합 시연

```bash
# Pi
~/safecar_start.sh        # 통합 launch에 v2x_bridge_node 포함
ros2 topic echo /v2x/status     # A,1,<code>,... 가 1초마다 나오면 전송 중
```

뒤차 화면(ESP32-B + 넥션)은 USB 전원만 연결하면 된다. `driving_state`가 바뀌면 0.1초 안에 화면이 바뀐다.
