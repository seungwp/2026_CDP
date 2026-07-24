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
colcon build --packages-select stella_md safecar_control
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

### 2-A. 통합 런치 (권장)

가장 흔히 쓰는 3가지 구성. 하나만 골라 띄웁니다.

#### ① 자율주행 (차선 추종) — 메인

차선 인식(`vision_detector`) + 차선 추종(`lane_follower`) + 안전 게이트(`decision_maker`) + 센서 브릿지 + 카메라 + Hailo를 **한 번에** 띄웁니다.

```bash
ros2 launch safecar_bringup safecar.launch.py lane_follow:=true anomaly_delay_sec:=-1.0
```

launch 인자:

| 인자 | 기본값 | 의미 |
|---|---|---|
| `lane_follow` | `false` | `true`면 차선 인식·추종 노드 실행(teleop 불필요) |
| `anomaly_delay_sec` | `10.0` | N초 후 운전자 이상(bio_anomaly=True) 시뮬레이션 → 갓길 대피(MRM) 발동. **`-1.0`이면 비활성**(정상 주행만) |

- 자율주행 순수 테스트: `lane_follow:=true anomaly_delay_sec:=-1.0`
- 갓길 대피(MRM) 데모: `lane_follow:=true anomaly_delay_sec:=10.0` → 10초 뒤 우측 갓길로 감속·정차
- `lane_follow:=false`로 두고 teleop을 `/cmd_vel_raw`로 쏘면, 게이트를 거치는 수동주행도 가능(장애물 자동정지·타임아웃 살아있음)

> ⚠️ teleop과 `lane_follow:=true`를 **동시에 켜지 말 것** — 둘 다 `/cmd_vel_raw`에 publish해서 명령이 섞입니다.

#### ② 수동 주행 (teleop) + 촬영

안전 게이트·Hailo·차선 노드 없이 **차체 구동부 + 카메라만** 띄웁니다. 게이트가 없으므로 teleop은 `/cmd_vel`로 직접 publish합니다(타임아웃 스톱-고 현상 없음).

```bash
ros2 launch safecar_bringup manual_drive.launch.py
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
ros2 launch ydlidar_ros    ydlidar_launch.py               # YDLIDAR X4만 (/scan 발행)
ros2 launch stella_bringup stella_state_publisher.launch.py # robot_state_publisher (URDF/TF)
ros2 launch stella_hailo_rpi5_ros2_examples hailo_ros2_detection_launch.py  # Hailo 객체인식만
```

---

### 2-C. 개별 노드 실행 (`ros2 run`) — 디버깅용

통합 런치 없이 노드를 하나씩 띄워 격리 테스트할 때. (파라미터는 `--ros-args -p 이름:=값`)

| 패키지 | 실행 노드 | 역할 |
|---|---|---|
| `safecar_perception` | `vision_detector_node` | OpenCV 차선 인식 → `/perception/lane_offset` |
| `safecar_perception` | `ufld_hailo_node` | (대안) UFLD 딥러닝 차선 인식 — HEF 필요 |
| `safecar_perception` | `mock_obstacle_node` | 장애물 없음(False) 상시 발행 — Hailo 대체 |
| `safecar_control` | `lane_follower_node` | 오프셋 → 조향(`/cmd_vel_raw`) |
| `safecar_control` | `decision_maker_node` | 안전 게이트(`/cmd_vel_raw`→`/cmd_vel`) |
| `safecar_comms` | `sensor_bridge_node` | 생체신호 브릿지(`/sensors/bio_anomaly`) |
| `stella_hailo_rpi5_ros2_examples` | `hailo_ros2_detection_node` | Hailo NPU 객체 인식 |
| `stella_teleop_bluetooth` | `stella_teleop_bluetooth_node` | 블루투스 조이패드 teleop |

예시:

```bash
# 차선 인식만 단독 실행 + 오프셋 확인
ros2 run safecar_perception vision_detector_node
ros2 topic echo /perception/lane_offset

# 차선 추종 노드를 파라미터 오버라이드로 실행
ros2 run safecar_control lane_follower_node --ros-args -p steer_gain:=0.7 -p cruise_speed:=0.1
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
# 디버그 영상: 원격 PC(rqt/rviz)에서 /perception/lane_image 확인
```

---

### 2-F. 종료

- `Ctrl+C`로 런치를 종료합니다.
- **모터 워치독**: 마지막 `/cmd_vel` 이후 0.5초가 지나면 `stella_md`가 바퀴에 정지 명령을 자동으로 보냅니다. 즉 Ctrl+C로 스택을 죽여도 차량이 스스로 멈춥니다. (이 워치독이 없으면 마지막 속도가 하드웨어에 래치돼 계속 굴러감 — 과거 버그였고 이번에 수정됨)
- 수동주행(2-A②)은 게이트/워치독 경로가 달라, teleop에서 `s`/`q`로 명시적으로 멈추는 걸 권장합니다.

---

## 3. 차선 추종 캘리브레이션 & 튜닝

인지: 버드아이(원근 변환) + sliding-window 다항식 중심선 → `/perception/lane_path`(정규화)
제어: Pure Pursuit + 곡률 감속 + rate-limit (`lane_follower_node.py`)

대부분 ROS 파라미터라 **재빌드 없이** launch 파일 수정 후 relaunch(또는 `-p` 오버라이드)로 조정합니다.
표시된 것만 코드 상수(수정 시 재빌드 필요). 값은 `safecar_bringup/launch/safecar.launch.py`의
`vision_detector_node`/`lane_follower_node` parameters 블록에서 바꿉니다.

### 3-0. 원근 캘리브레이션 (제일 먼저, 필수)

`vision_detector_node`의 `persp_src`(사다리꼴 4점, 640×480 기준)를 조정합니다.

```
persp_src = [tl_x,tl_y, tr_x,tr_y, br_x,br_y, bl_x,bl_y]   # 기본: [200,280, 440,280, 600,470, 40,470]
```

절차:
1. `ros2 launch safecar_bringup safecar.launch.py lane_follow:=true anomaly_delay_sec:=-1.0`
2. 차를 **직선 차선** 위에 두고, 원격 PC(rqt/rviz)에서 `/perception/lane_image` 확인.
3. 파란 사다리꼴(=persp_src)이 좌우 차선을 감싸고, 초록(차선)·노랑(중심선) 선이 차선과 잘 겹치도록 4점을 조정.
4. 직선 구간에서 중심선이 **곧게** 그려지면 캘리브레이션 완료.

### 3-1. 직선 안정성 (흔들림 제거) — `lane_follower_node`

| 파라미터 | 기본값 | 역할 | 증상 → 방향 |
|---|---|---|---|
| `lookahead_dist` | 0.8 | Pure Pursuit 전방 목표 거리(정규화, 최대 ~0.97) | **흔들리면 ↑** / 코너 컷·둔하면 ↓ (영향 제일 큼) |
| `steer_gain` | 0.8 | 전체 조향 세기 | 흔들리면 ↓(0.6~) / 굼뜨면 ↑ |
| `steer_deadband` | 0.05 | 이 이내 횡오차 무시 | 중앙 근처 떨리면 ↑ |
| `max_ang_accel` | 4.0 | 조향 급변 제한(rad/s²) | 명령이 홱홱하면 ↓ |
| `max_steer` | 1.0 | 조향 각속도 상한(rad/s) | 과조향 클램프 |

### 3-2. 곡선 (미리 조향·자동 감속) — `lane_follower_node`

| 파라미터 | 기본값 | 역할 | 증상 → 방향 |
|---|---|---|---|
| `k_curv` | 1.0 | 곡률 기반 감속 세기 | 코너에서 밀려나면(오버슈트) ↑ / 너무 느리면 ↓ |
| `lookahead_dist` | 0.8 | (재조정) 곡선 컷 vs 오버슈트 균형 | 안쪽 파고들면 ↑ |
| `cruise_speed` | 0.12 | 기본 전진 속도(m/s) | 전체 속도감 |
| `v_min` | 0.06 | 곡선 감속 하한(m/s) | 곡선에서 너무 느려 멈추면 ↑ |

### 3-3. 스무스 갓길 대피(MRM) — `lane_follower_node`

| 파라미터 | 기본값 | 역할 | 증상 → 방향 |
|---|---|---|---|
| `mrm_transition_time` | 2.0 | 갓길로 붙는 시간(호의 완만함) | 급격하면 ↑(2~3s) |
| `mrm_target_offset` | -0.6 | 갓길 목표 위치(음수=우측, 정규화) | 덜 붙으면 더 음수로 |
| `mrm_speed_ratio` | 0.6 | 대피 중 속도 비율 | — |
| `mrm_stop_duration` | 3.0 | 갓길 도달 후 정차까지 시간(초) | — |

### 3-4. 조명/차선색 바뀔 때 (코드 상수, 재빌드 필요)

차선을 **아예 못 잡으면** 여기부터. `vision_detector_node.py`의 `VisionDetector._color_mask` HSV 임계값
(노랑 `inRange([18,70,70]~[40,255,255])`, 흰색 `inRange([0,0,200]~[180,25,255])`, `USE_WHITE`)을
트랙 조명·차선색에 맞춰 조정합니다.

> 튜닝 순서 원칙: **3-0(캘리브레이션) → 3-1(직선) → 3-2(곡선) → 3-3(MRM)**. 앞 단계가 안정돼야 뒤가 의미 있습니다.
>
> 확인용 토픽: `ros2 topic echo /perception/lane_offset` (횡오차 -1~+1), `ros2 topic echo /perception/lane_path` (정규화 중심선 [lat,fwd,...]).

---

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
cp safecar/safecar_control/safecar_control/lane_follower_node.py ~/lane_follower_pi_local.py

# 2) Pi에서 뭘 바꿨는지 확인
git diff safecar/safecar_control/safecar_control/lane_follower_node.py

# 3) 원격 버전 채택(로컬 수정 버림) 후 pull  ※ 백업이 있으니 안전
git checkout -- safecar/safecar_control/safecar_control/lane_follower_node.py
git pull

# 4) 트랙에서 검증한 튜닝값만 재적용 (예시)
sed -i "s/'steer_d_gain', 0.3/'steer_d_gain', 0.5/" \
  safecar/safecar_control/safecar_control/lane_follower_node.py
sed -i "s/'offset_smoothing', 0.8)/'offset_smoothing', 0.85)/" \
  safecar/safecar_control/safecar_control/lane_follower_node.py

# 5) 확인 후 재빌드
grep -nE "steer_gain|steer_d_gain|steer_deadband|max_steer|offset_smoothing" \
  safecar/safecar_control/safecar_control/lane_follower_node.py
colcon build --packages-select safecar_control && source install/setup.bash
```

> Pi에서 반복적으로 로컬 수정 → pull 충돌이 나는 것을 막으려면, **검증된 튜닝값을 PC 저장소 기본값에 반영해 commit/push** 해두는 게 좋습니다. 그러면 Pi는 로컬 수정 없이 깨끗하게 pull됩니다.

---

## 참고

- 아키텍처·토픽 계약: [`docs/ARCHITECTURE.md`](./ARCHITECTURE.md)
- 안전 게이트 우선순위: 전방 장애물(정지/회피) → 운전자 이상(갓길 대피) → 정상 주행
