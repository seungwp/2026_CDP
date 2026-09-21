# 통신부 (safecar_comms) — 운전자 이상신호 인터페이스

담당: 정수영 · 노드 구현 레포: **`cdp-remotepc`** (우분투 VM에서 실행)

> 📐 **감지 알고리즘 스펙은 [`docs/DRIVER_MONITORING_SPEC.md`](../../docs/DRIVER_MONITORING_SPEC.md)** 에 따로 있다.
> (규격 근거 · 개인 캘리브레이션 · PERCLOS/KSS · 상태머신 · 검증 시나리오)
> 이 문서는 **Pi와의 접점(토픽 계약)** 만 다룬다.

## 작업 경계

운전자 이상 감지(웹캠)는 **노트북 VM에서 돌고, 코드도 `cdp-remotepc` 레포에 있다.**
이 레포(`2026_CDP` / Pi 워크스페이스)와의 접점은 **토픽 하나뿐**이다.

```
[VM · cdp-remotepc]                        [Pi · 2026_CDP]
 웹캠 → driver_monitor_node  ──/sensors/bio_anomaly──▶  decision_maker_node → MRM 갓길 정차
                                    (std_msgs/Bool)
```

- 양쪽은 같은 `ROS_DOMAIN_ID=52`만 맞으면 DDS가 알아서 연결한다. 별도 통신 계층 없음.
- **VM 쪽은 이 레포를 건드리지 않는다** → Pi 재빌드 불필요.
- **Pi 쪽은 VM 노드 없이도 개발·테스트한다** (아래 독립 테스트 참고).

## 토픽 계약 (이것만 지키면 됨)

| 항목 | 값 |
|---|---|
| 토픽 | `/sensors/bio_anomaly` |
| 타입 | `std_msgs/Bool` |
| 방향 | VM(publish) → Pi(subscribe) |
| QoS | 기본(RELIABLE, depth 10) |
| 발행 주기 | **2Hz 이상으로 계속** 발행 (이벤트 1회성 X) |

의미 규약:

- `data=False` — 운전자 정상. **정상일 때도 계속 발행**해야 한다(발행이 끊기면 Pi는 마지막 값을 그대로 유지).
- `data=True` — 운전자 이상. Pi가 즉시 `MRM_PULL_OVER`로 전환해 갓길 정차를 시작한다.
- **⚠️ latch(상태 유지)는 publisher 책임** — 한 프레임이라도 `True→False`로 튀면 차가 갓길에서 **다시 출발한다**.
  `True`로 한 번 올렸으면 명확한 복구 조건(예: 눈 뜬 상태 3초 유지)이 성립할 때까지 계속 `True`를 유지할 것.
  (Pi 쪽에도 방어용 latch를 추가할 예정이지만, 1차 책임은 publisher에 있다.)

## 금지사항

- **`/cmd_vel`에 직접 publish 금지.** 바퀴로 나가는 명령은 `decision_maker_node`만 발행한다(단일 안전 게이트).
- **시뮬레이션과 동시 실행 금지.** 이 패키지의 `sensor_bridge_node`도 같은 토픽을 발행하므로
  둘이 동시에 뜨면 True/False가 번갈아 들어와 차가 멈췄다 갔다 한다.
  → 실제 VM 노드를 쓸 때는 `anomaly_delay_sec:=-1.0`(현재 기본값)으로 시뮬을 끈 상태로 둔다.

## 각자 독립 테스트

**VM 쪽 (Pi 없이)** — 발행이 규약대로 나오는지만 확인:

```bash
export ROS_DOMAIN_ID=52
ros2 topic echo /sensors/bio_anomaly     # 정상 False 연속 → 이상 시 True로 latch 되는지
ros2 topic hz   /sensors/bio_anomaly     # 2Hz 이상 유지되는지
```

**Pi 쪽 (VM 노드 없이)** — 손으로 신호를 쏴서 MRM 동작 검증:

```bash
ros2 topic pub -r 2 /sensors/bio_anomaly std_msgs/msg/Bool "{data: true}"
ros2 topic echo /control/driving_state   # NORMAL → MRM_PULL_OVER 전환 확인
```

## 통합 시연 실행 순서

```bash
# 1) Pi
ros2 launch safecar_bringup safecar.launch.py lane_follow:=true
# 2) VM (ROS_DOMAIN_ID=52)
ros2 run <pkg> driver_monitor_node
# 3) VM에서 상태 모니터링
ros2 topic echo /control/driving_state
```

## 시연 영상 촬영 요구사항

차체가 작아 사람이 탑승할 수 없으므로, 운전자(노트북 앞)와 차량(트랙)은 **따로 촬영해 편집으로 붙인다.**
그래서 "운전자 이상 → 차량 반응"의 인과를 **VM 화면 하나로** 증명할 수 있어야 한다.

`driver_monitor_node`의 디버그 창에 아래를 같이 그려줄 것:

- 웹캠 영상 + 얼굴/눈 랜드마크
- 판정 수치(EAR 등)와 자체 상태 (`AWAKE` / `WARN` / `INCAPACITATED`)
- **`/control/driving_state` 구독값** (Pi가 판단한 `NORMAL` / `MRM_PULL_OVER` / `EMERGENCY_BRAKE`)

마지막 항목이 핵심이다 — 구독 한 줄 + `cv2.putText` 한 줄이면 되고, 이게 있어야
"눈 감음 → 차가 갓길로" 가 한 화면에서 증명된다. 차량 주행 영상은 폰으로 따로 찍어 편집 시 붙인다.

## `sensor_bridge_node` (이 패키지에 남아있는 것)

`anomaly_delay_sec`초 후 `True`를 내보내는 **시뮬레이션**. 실제 감지는 VM 노드가 담당하므로
이제 기본값은 `-1.0`(비활성)이다. VM 없이 Pi 단독으로 MRM을 데모할 때만 켠다:

```bash
ros2 launch safecar_bringup safecar.launch.py lane_follow:=true anomaly_delay_sec:=10.0
```
