# telemetry — V2V 위험 경고 (ESP32 + 넥션)

우리 차의 주행 상태(정상 / 급제동 / 운전자 이상 비상정차)를 ESP-NOW로 방송해
뒤차 운전석 화면(넥션)에 경고로 띄운다.
신호 규약과 Pi 설정은 [`docs/V2X_SIGNAL_CONTRACT.md`](../docs/V2X_SIGNAL_CONTRACT.md) 참고.

담당: 정수영

## 폴더

| 경로 | 내용 |
|---|---|
| `esp32_vehicle/` | **ESP32-A (차량용, MAC `...:4C`)** — Pi 시리얼 `S,<code>,<speed>` → BSM 방송 |
| `esp32_display/` | **ESP32-B (뒤차 화면용, MAC `...:78`)** — BSM 수신 → 넥션 `page 0~3` |
| `nextion/` | 넥션 화면 프로젝트 `safecar_v2x.HMI` / 기기용 `safecar_v2x.tft` / 배경 그림 |
| `nextion/legacy/` | 초기 화면 (전방·후방 이상감지 3페이지) |
| `dev/pi_sim_sender.py` | Pi 없이 노트북에서 ESP32-A로 상태를 보내는 테스트 도구 |
| `dev/c2_tx_test/` | 위험코드 0→3을 2초마다 돌리는 하드웨어 확인용 스케치 |
| `dev/initial_espnow/` | 처음 만든 ESP-NOW 송수신 코드 (기록용) |

## 펌웨어 올리기 (Arduino IDE)

- 보드 패키지: **esp32 by Espressif Systems 3.x** (수신 콜백이 `esp_now_recv_info_t`를 씀)
- 보드: `ESP32 Dev Module`, 시리얼 모니터 115200
- 업로드 후 시리얼 모니터 첫 줄의 MAC 끝자리로 A/B가 맞게 올라갔는지 확인
  - A: `# esp32_vehicle ready  ID=1  MAC=...:4C`
  - B: `# esp32_display ready  target ID=1  MAC=...:78`

## 넥션 화면 올리기

`nextion/safecar_v2x.tft` 하나만 FAT32 microSD 카드에 복사 → 넥션 전원 끈 상태로 꽂고 켜기 →
`Update Successed!` 후 전원 끄고 카드 빼기.

| page | 화면 |
|---|---|
| 0 | 이상없음 |
| 1 | 전방차량 급제동 |
| 2 | 전방차량 비상정차 |
| 3 | 전방차량 신호없음 |
