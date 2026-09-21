<!-- markdownlint-disable MD033 MD041 -->
# SafeCar 실행 & 복구 런북

라즈베리파이(Pi)에서 SafeCar를 빌드·실행하고, git 저장소가 깨졌을 때 복구하는 절차를 정리한 문서.

- 대상 차체: STELLA N1 (Raspberry Pi 5 + Hailo-8 + YDLIDAR X4)
- 미들웨어: ROS 2 Jazzy
- Pi 워크스페이스: `~/2026_CDP` (colcon workspace)
- 원격: `https://github.com/seungwp/2026_CDP.git` (branch `main`)

> ⚠️ **전원 주의**: Pi 전원 마진이 빠듯합니다(배터리 최저 ~4.80V, 헤드룸 0). **빌드·git 작업은 반드시 충전기(벤치 전원)에 물린 상태**에서 하세요. 쓰기 도중 전압 강하가 나면 git 객체가 깨집니다(아래 "복구" 참고).

---

## 1. 빌드

Pi는 `--symlink-install`을 쓰지 않으므로, **파이썬 노드를 고쳐도 반드시 `colcon build`** 를 다시 해야 반영됩니다.

```bash
cd ~/2026_CDP

# 변경한 패키지만 빌드 (빠름)
colcon build --packages-select safecar stella_md
source install/setup.bash
```

전체 빌드가 필요하면:

```bash
cd ~/2026_CDP
colcon build
source install/setup.bash
```

---

## 2. 실행 방법 (전체)

> 모든 터미널에서 먼저 환경을 잡아야 합니다.
> ```bash
> cd ~/2026_CDP
> source install/setup.bash
> export ROS_DOMAIN_ID=52
> ```
> `ROS_DOMAIN_ID=52`는 원격 PC(제어 스테이션)와 토픽을 주고받기 위한 값입니다. Pi 단독 실행에도 붙여두면 됩니다.

### 토폴로지 한눈에

```
카메라 ─▶ vision_detector ─▶ /perception/lane_offset ─▶ lane_follower ─▶ /cmd_vel_raw
                                                                              │
 bio_anomaly, obstacle_detected ─▶ decision_maker (안전 게이트) ◀────────────┘
                                        │
                                        ▼  /cmd_vel
                                     stella_md (모터) ── 워치독: 0.5s 끊기면 자동 정지
```

---

### 2-A0. 스크립트로 실행 (제일 간단)

Pi 홈에 있는 스크립트 4개로 대부분의 작업이 됩니다(원본은 리포의 `scripts/pi/`).

```bash
~/safecar_start.sh              # 차체+카메라+차선인식+게이트+영상 전부 켜기
~/safecar_drive.sh              # 주행 시작 (Ctrl+C로 정지)
~/safecar_drive.sh 0.15 0.7 0.4 # 속도 / 조향게인 / 헤딩게인 바꿔서
~/safecar_stop.sh               # 비상 정지 (제어만)
~/safecar_stop.sh all           # 카메라까지 전부 끔
```

> ⚠️ **`ros2 run`의 PID만 죽이면 실제 노드가 살아남습니다.** 백그라운드로 띄운 주행을
> 래퍼 PID로 종료했다가 차가 계속 달린 적이 있습니다(12.8m). `safecar_stop.sh`는
> 노드를 이름으로 직접 죽이고 정지 명령까지 발행하므로 이걸 쓰세요.

### 2-A. 통합 런치 (권장)

가장 흔히 쓰는 3가지 구성. 하나만 골라 띄웁니다.

#### ① 자율주행 (차선 추종) — 메인

차선 인식(`vision_detector`) + 차선 추종(`lane_follower`) + 안전 게이트(`decision_maker`) + 센서 브릿지 + 카메라 + Hailo를 **한 번에** 띄웁니다.

```bash
ros2 launch safecar safecar.launch.py lane_follow:=true anomaly_delay_sec:=-1.0
```

launch 인자:

| 인자 | 기본값 | 의미 |
|---|---|---|
| `lane_follow` | `false` | `true`면 차선 인식·추종 노드 실행(teleop 불필요) |
| `anomaly_delay_sec` | `-1.0` | N초 후 운전자 이상(bio_anomaly=True) 시뮬레이션 → 갓길 대피(MRM) 발동. **`-1.0`이면 비활성**(정상 주행만) |

- 자율주행 순수 테스트: `lane_follow:=true anomaly_delay_sec:=-1.0`
- 갓길 대피(MRM) 데모: `lane_follow:=true anomaly_delay_sec:=10.0` → 10초 뒤 우측 갓길로 감속·정차
- `lane_follow:=false`로 두고 teleop을 `/cmd_vel_raw`로 쏘면, 게이트를 거치는 수동주행도 가능(장애물 자동정지·타임아웃 살아있음)

> ⚠️ teleop과 `lane_follow:=true`를 **동시에 켜지 말 것** — 둘 다 `/cmd_vel_raw`에 publish해서 명령이 섞입니다.

#### ② 수동 주행 (teleop) + 촬영

안전 게이트·Hailo·차선 노드 없이 **차체 구동부 + 카메라만** 띄웁니다. 게이트가 없으므로 teleop은 `/cmd_vel`로 직접 publish합니다(타임아웃 스톱-고 현상 없음).

```bash
ros2 launch safecar manual_drive.launch.py
# 터미널 2: python3 ~/teleop.py     (/cmd_vel 로 직접 조종)
# 터미널 3: ~/record_drive.sh       (주행 영상 녹화)
```

> 안전 게이트가 없어 장애물 자동정지·명령 타임아웃이 동작하지 않습니다. 멈추려면 teleop에서 `s`(정지) 또는 `q`(종료 시 정지 명령 발행)를 쓰세요.

#### ③ 차체 구동부만

모터드라이버(`stella_md`) + IMU(`stella_ahrs`) + YDLIDAR + 상태 퍼블리셔만. 카메라·인지·게이트 없음. 하드웨어 점검·odom/tf·라이다 확인용.

```bash
ros2 launch stella_bringup robot.launch.py
```

---

### 2-B. 개별 하드웨어 런치 (부분 점검)

`robot.launch.py`가 내부에서 include하는 것들. 특정 장치만 따로 띄워 점검할 때.

```bash
ros2 launch stella_md      stella_md_launch.py             # 모터드라이버만 (/cmd_vel 구독, /odom 발행)
ros2 launch stella_ahrs    stella_ahrs_launch.py           # IMU/AHRS만 (imu/yaw 발행)
ros2 launch ydlidar        ydlidar_launch.py               # YDLIDAR X4만 (/scan 발행)
ros2 launch stella_hailo_rpi5_ros2_examples hailo_ros2_detection_launch.py  # Hailo 객체인식만
```

---

### 2-C. 개별 노드 실행 (`ros2 run`) — 디버깅용

통합 런치 없이 노드를 하나씩 띄워 격리 테스트할 때. (파라미터는 `--ros-args -p 이름:=값`)

| 패키지 | 실행 노드 | 역할 |
|---|---|---|
| `safecar` | `vision_detector_node` | OpenCV 차선 인식 → `/perception/lane_offset` |
| `safecar` | `lane_follower_node` | 오프셋 → 조향(`/cmd_vel_raw`) |
| `safecar` | `decision_maker_node` | 안전 게이트(`/cmd_vel_raw`→`/cmd_vel`) |
| `safecar` | `sensor_bridge_node` | 생체신호 브릿지(`/sensors/bio_anomaly`) |
| `stella_hailo_rpi5_ros2_examples` | `hailo_ros2_detection_node` | Hailo NPU 객체 인식 |

예시:

```bash
# 차선 인식만 단독 실행 + 오프셋 확인
ros2 run safecar vision_detector_node
ros2 topic echo /perception/lane_offset

# 차선 추종 노드를 파라미터 오버라이드로 실행
ros2 run safecar lane_follower_node --ros-args -p steer_gain:=0.7 -p cruise_speed:=0.1
```

> 참고: 노드는 `__init__`에서 파라미터를 한 번만 읽으므로, 실행 중 `ros2 param set`은 즉시 반영되지 않습니다. 값 바꾸려면 재실행(또는 재빌드)하세요.

---

### 2-D. 보조 스크립트 (Pi 홈 디렉터리)

리포에 없고 Pi `~/`에 있는 헬퍼입니다.

```bash
python3 ~/teleop.py       # 키보드 teleop → /cmd_vel (manual_drive용)
~/record_drive.sh         # /camera/image_raw 를 mp4로 녹화 (camera_node가 CSI 점유하므로 rpicam 불가)
```

---

### 2-E. 상태 점검 (실행 중)

```bash
ros2 node list                              # 떠 있는 노드
ros2 topic list                             # 토픽 목록
ros2 topic echo /control/driving_state      # 게이트 상태(NORMAL/MRM_PULL_OVER/EMERGENCY_BRAKE)
ros2 topic echo /perception/lane_offset     # 차선 오프셋(-1~+1)
ros2 topic echo /cmd_vel                    # 실제 바퀴로 나가는 명령
ros2 topic echo /cmd_vel_raw                # 게이트 이전 주행 명령
ros2 topic hz /camera/image_raw             # 카메라 프레임레이트
# 디버그 영상: 브라우저로 보기(VM 불필요) — 아래 2-G 참고
#            또는 원격 PC(rqt/rviz)에서 /perception/lane_image 확인
```

---

### 2-G. 진단 스크립트 (`scripts/` — VM 없이 SSH만으로)

`colcon build` 불필요. 환경만 잡고 `python3`로 바로 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/camera_ws/install/local_setup.bash      # camera_ros
source ~/jazzy_ws/install/local_setup.bash       # cv_bridge
source ~/2026_CDP/install/local_setup.bash
export ROS_DOMAIN_ID=52
```

**카메라 영상을 노트북 브라우저로 보기** — 우분투 VM도 rqt도 필요 없습니다.

```bash
python3 scripts/image_server.py                            # /perception/lane_image (차선 디버그)
python3 scripts/image_server.py --topic /camera/image_raw  # 원본 (카메라 물리 조준용)
```

노트북 브라우저에서 **`http://raspberrypi.local:8080/`** — 라이브 영상.
Pi IP는 공유기가 DHCP로 주는 값이라 바뀝니다. mDNS 이름(`raspberrypi.local`)을 쓰면 IP를 몰라도 됩니다.
`/snapshot`으로 정지 프레임 1장을 받을 수 있습니다(튜닝 근거 이미지용).

**라이다 섹터 점검** — 가림 확인 + 각도 실측 (3-4 참고)

```bash
python3 scripts/scan_check.py
python3 scripts/scan_check.py --selftest   # ROS 없이 로직만 검증
```

---

### 2-F. 종료

- `Ctrl+C`로 런치를 종료합니다.
- **`camera_node`는 SIGTERM으로 안 죽고 CSI를 물고 있습니다** — 다음 실행이 "Pipeline handler in use by another process"로 실패하므로 반드시:
  ```bash
  pkill -9 -f camera_node; pkill -f vision_detector_node; pkill -f image_server
  ```
- **모터 워치독**: 마지막 `/cmd_vel` 이후 0.5초가 지나면 `stella_md`가 바퀴에 정지 명령을 자동으로 보냅니다. 즉 Ctrl+C로 스택을 죽여도 차량이 스스로 멈춥니다. (이 워치독이 없으면 마지막 속도가 하드웨어에 래치돼 계속 굴러감 — 과거 버그였고 이번에 수정됨)
- 수동주행(2-A②)은 게이트/워치독 경로가 달라, teleop에서 `s`/`q`로 명시적으로 멈추는 걸 권장합니다.

---

## 3. 차선 추종 & 갓길 대피 튜닝

인지: OpenCV HSV 노란 마스크 + Canny + HoughLinesP → `/perception/lane_offset`(-1~+1)
제어: P 조향 + 오프셋 EMA (`lane_follower_node`), 갓길 대피는 `mrm_profile.MrmProfile`

> 2026-07-24에 시도했던 버드아이(원근변환) + Pure Pursuit은 트랙에서 흔들려 **되돌렸습니다**.
> 그때 쓰던 `persp_src` / `lookahead_dist` / `k_curv` 같은 파라미터는 **지금 코드에 없습니다**.
> 코드는 git 히스토리에 남아 있고, 쓰려면 카메라를 더 숙여 단 뒤 재캘리브레이션해야 합니다.

### 3-1. 인지 상수 (코드 상수 — 수정 시 재빌드 필요)

`safecar/safecar/perception/vision_detector.py`. **2026-09-21 실외 아스팔트 트랙(흰 점선) 실측값**입니다.
조명·노면·카메라 각도가 바뀌면 다시 잡아야 합니다.

| 상수 | 값 | 근거 / 조정 방향 |
|---|---|---|
| `FOLLOW_SINGLE_LINE` | `True` | 점선 중앙선 한 줄을 직접 추종. 차로가 여러 개면 좌/우 짝짓기가 오히려 불안정 |
| `USE_WHITE` / `USE_YELLOW` | `True` / `False` | 실외 흰 차선. **실내 노란 테이프 트랙이면 반대로** |
| `WHITE_V_MIN` | `200` | 230으로 올리면 햇빛 잡음은 줄지만 **멀리 있는 어두운 점선이 먼저 잘린다**(실측: 점선 밝기가 y=400에서 248, y=460에서 195) |
| `ROI_TOP` | `0.35` | 점선 조각 사이 공백을 넘기려면 다음 조각까지 보여야 함 |
| `Y_EVAL` | `0.6` | 작을수록 멀리 봐서 곡선 선제 대응. 대신 오프셋이 작아져 게인을 올려야 함 |
| `MIN_ABS_SLOPE` | `0.8` | 실제 차선은 기울기 1.5~2.5, 노면 얼룩은 1.0 이하 |
| `MAX_ABS_OFFSET` | `0.7` | 넘으면 유실 처리. **없으면 잡음을 쫓아 코스를 벗어난다(실측 12.8m 폭주)** |
| `MAX_JUMP_PX` | `45` | 프레임당 허용 이동. 점선 공백에서 **옆 차선으로 갈아타는 것을 막는다** |
| `MAX_COAST_FRAMES` | `18` | 공백 구간을 직전 값으로 버티는 프레임 수(≈0.4초) |

### 3-2. 주행 제어 (`lane_follower_node` 파라미터 — 재빌드 불필요)

| 파라미터 | 값 | 실측 근거 |
|---|---|---|
| `cruise_speed` | `0.15` | **0.10 이하는 정지 마찰을 못 이겨 아예 안 움직인다.** 0.12부터 구동 |
| `steer_gain` | `0.55` | **1.2는 발산 진동**(조향 0.98 rad/s = 회전반경 8cm). 0.4~0.55에서 안정 |
| `steer_head_gain` | `0.25` | 곡선 선제 조향(Stanley의 헤딩 항). 곡선에서 밀리면 ↑, 급하면 ↓ |
| `offset_smoothing` | `0.7` | 검출 노이즈가 조향에 실리는 것을 막음 |

30초 주행 실측(gain 0.55): 4.33m, 오프셋 평균 +0.031, 최대 조향 0.21 rad/s, 유실 0회.

> **곡선이 안 되면** `steer_head_gain`부터 올리세요. 횡오차 게인만 올리면 직선에서 흔들립니다.
> 횡오차는 차가 이미 밀린 뒤에야 생기지만, 헤딩은 밀리기 전에 곡선을 알려줍니다.

### 3-3. 갓길 대피(MRM)

**모드 결정** — 선행특허 KR 10-2024-0073259의 결정 플로우를 센서 구성에 맞춰 구현했습니다.

```
차로 유지 가능? ─아니오─▶ 직진정차 (조향 끊고 감속만)
     │예
차로 변경 가능? ─아니오─▶ 자차로정차
     │예
                      └─▶ 우차로정차
```

| 파라미터 | 기본값 | 역할 |
|---|---|---|
| `mrm_lateral_bias` | 0.5 | 대피 목표 횡위치. **양수=우측**. `0.0`이면 차로 내 정지(R157 기본) |
| `mrm_transition_time` | 3.0 | 갓길로 붙는 시간(초) |
| `mrm_speed_ratio` | 0.6 | 이동 중 속도 비율 |
| `mrm_stop_duration` | 2.0 | 붙은 뒤 정지까지(초) |

**라이다 기반 대피 가능 판정**

| 파라미터 | 기본값 | 역할 |
|---|---|---|
| `mrm_rear_deg` / `mrm_side_deg` | 180.0 / -90.0 | 후방·우측 섹터 중심각. **실측 확인 필수**(3-4) |
| `mrm_sector_half_deg` | 30.0 | 섹터 반각 |
| `mrm_rear_clear_m` / `mrm_side_clear_m` | 0.6 / 0.35 | 이보다 비어야 횡이동 허용 |
| `mrm_require_scan` | True | `/scan` 없으면 자차로정차로 폴백. 라이다 없이 튜닝할 땐 False |

`decision_maker_node`:

| 파라미터 | 기본값 | 역할 |
|---|---|---|
| `bio_latch` | True | 운전자 이상 래치(R157: 정차 후 수동 입력 전 재출발 금지). **튜닝 중엔 False** |
| `cmd_vel_timeout` | 1.0 | `/cmd_vel_raw` 끊김 판정(초) |

> ⚠️ 벽 가까이에서 테스트하면 우측이 막혀 **항상 `자차로정차`** 가 나옵니다. 버그가 아닙니다.
> MRM 모드 테스트는 트인 곳에서 하세요.

### 3-4. 라이다 섹터 각도 실측 (한 번만)

`ydlidar.yaml`에 `reversion: true`가 걸려 있어 **ROS 표준 각도와 다를 수 있습니다.**
`mrm_side_deg`가 틀리면 갓길 판정이 통째로 틀어지므로 반드시 확인합니다.

```bash
ros2 launch ydlidar ydlidar_launch.py      # 터미널 1
python3 scripts/scan_check.py              # 터미널 2 (colcon build 불필요)
```

1. **빈 공간에서 실행** → 특정 칸만 항상 짧으면 차체/구조물에 가린 것
2. **차 우측에만 물체를 두고** → `<== 최근접`이 뜨는 칸의 **중심각**이 `mrm_side_deg`
3. 뒤에만 물체 → 같은 방법으로 `mrm_rear_deg`

## 4. git 저장소 복구 (loose object 깨짐)

전원 차단 등으로 `git status` / `git pull`이 아래처럼 실패할 때:

```
error: object file .git/objects/xx/xxxx... is empty
fatal: loose object xxxx... is corrupt
```

### 4-1. 피해 범위 확인

```bash
cd ~/2026_CDP
git fsck --full     # empty/corrupt 객체 목록
```

### 4-2. 원격에서 재수신 (권장)

깨진 커밋들이 이미 원격(origin)에 올라가 있으면 다시 받아 복구할 수 있습니다.
일반 `git fetch`는 "이미 있음"으로 착각해 건너뛸 수 있으니 **`--refetch`(git 2.36+)** 로 강제 재다운로드합니다.

```bash
cp -r .git ../2026_CDP_git_backup          # 안전 백업
find .git/objects -type f -empty -delete   # 0바이트 객체 제거
git fetch origin --refetch                 # 전체 객체 강제 재수신
git fsck --full                            # 이제 깨끗해야 함
```

### 4-3. 그래도 안 되면: 새로 clone

객체 수술이 꼬이면 새로 받는 게 가장 확실합니다. **기존 폴더의 작업트리 파일은 그대로 남으니** 커밋 안 한 로컬 수정은 거기서 꺼내옵니다.

```bash
cd ~
git clone https://github.com/seungwp/2026_CDP.git 2026_CDP_new
# 필요한 Pi 로컬 수정분을 옛 폴더에서 복사한 뒤, 새 폴더에서 colcon build
```

---

## 5. pull 충돌 시 (로컬 수정 vs 원격 변경)

`git pull`이 "Your local changes ... would be overwritten by merge"로 막힐 때.
**Pi에서 직접 튜닝한 파라미터**를 잃지 않도록, 원격 버전을 받고 **검증된 값만 다시 얹는** 방식을 씁니다.

```bash
# 1) 로컬 수정 파일 백업
cp safecar/safecar/control/lane_follower_node.py ~/lane_follower_pi_local.py

# 2) Pi에서 뭘 바꿨는지 확인
git diff safecar/safecar/control/lane_follower_node.py

# 3) 원격 버전 채택(로컬 수정 버림) 후 pull  ※ 백업이 있으니 안전
git checkout -- safecar/safecar/control/lane_follower_node.py
git pull

# 4) 트랙에서 검증한 튜닝값만 재적용 (예시)
sed -i "s/'steer_d_gain', 0.3/'steer_d_gain', 0.5/" \
  safecar/safecar/control/lane_follower_node.py
sed -i "s/'offset_smoothing', 0.8)/'offset_smoothing', 0.85)/" \
  safecar/safecar/control/lane_follower_node.py

# 5) 확인 후 재빌드
grep -nE "steer_gain|steer_d_gain|steer_deadband|max_steer|offset_smoothing" \
  safecar/safecar/control/lane_follower_node.py
colcon build --packages-select safecar && source install/setup.bash
```

> Pi에서 반복적으로 로컬 수정 → pull 충돌이 나는 것을 막으려면, **검증된 튜닝값을 PC 저장소 기본값에 반영해 commit/push** 해두는 게 좋습니다. 그러면 Pi는 로컬 수정 없이 깨끗하게 pull됩니다.

---

## 참고

- 아키텍처·토픽 계약: [`docs/ARCHITECTURE.md`](./ARCHITECTURE.md)
- 안전 게이트 우선순위: 전방 장애물(정지/회피) → 운전자 이상(갓길 대피) → 정상 주행
