<!-- markdownlint-disable MD033 MD041 -->
<div align="center">

# 🚗 SafeCar

**운전자 이상·전방 장애물을 감지해 스스로 안전 조치를 취하는 자율주행 안전 감독(fail-safe) 시스템**

STELLA N1 차체 · Raspberry Pi 5 · Hailo-8 NPU 기반 ROS 2 자율주행 플랫폼

</div>

---

## 📌 프로젝트 정보

| 항목 | 내용 |
|---|---|
| **개발 기간** | 2026.05.07 ~ 2026.09.30 |
| **소속 / 과목** | 영남대학교 2026 융합 Capstone Design Project |
| **팀 구성** | 4인 (인지 · 제어 · 통신 · 대시보드) |
| **기반 플랫폼** | STELLA N1 (Raspberry Pi 5 + Hailo-8 AI HAT + YDLIDAR X4) |
| **미들웨어** | ROS 2 Jazzy |

---

## 👥 팀 소개

| 파트 | 담당 | 주요 패키지 |
|---|---|---|
| 🧭 제어 · 통합 | 김승제 | `safecar/safecar/control/`, `safecar/launch/` |
| 👁️ 인지 | 진다혜 | `safecar/safecar/perception/` |
| 📡 통신 | 정수영 | `safecar/safecar/comms/` + 노트북 웹캠 졸음 감지(`driver_monitor/`) |
| 📊 대시보드 | 성현서 | (미구현 — 패키지 정리됨) |


---

## 📖 프로젝트 소개

SafeCar는 일반 자율주행 스택 위에 **"안전 감독(supervisor) 레이어"** 를 얹은 프로젝트입니다.

- **문제의식** — 자율주행 중 ① 운전자의 갑작스러운 건강 이상, ② 전방 장애물, ③ 제어 스택 자체의 정지 같은 상황에서도 차량이 스스로 사고 없이 안전하게 대응해야 한다.
- **핵심 아이디어** — NTREX STELLA 차체 위에 **인지(카메라·NPU) → 판단(제어) → 통신(센서)** 3계층 SafeCar 레이어를 추가하고, 실제 바퀴로 나가는 `/cmd_vel`을 **단일 안전 게이트**로 통제한다. 나아가 **스택이 멈춰도 동작하는 차체 드라이버 워치독**으로 안전을 이중화한다.
- **주행은 두 방식을 상황에 따라 전환한다** — 평소엔 사람 조종 데이터로 학습한 **모방학습(behavior cloning) 모델**이 몰고, 운전자 이상이 감지되면 규칙기반 `lane_follower_node`가 조용히 이어받아 **갓길(노란 테이프) 실시간 인식**으로 붙여 정차시킨다. 두 노드가 같은 안전 게이트 상태를 보고 한쪽만 활성화되므로 서로 충돌하지 않는다.
- **3가지 안전 시나리오**
  - 🫀 **운전자 생체 이상** → 노트북 웹캠(모방학습 기반 얼굴 랜드마크)이 3초 이상 눈감김을 감지 → UDP로 신호 전달 → 감속하며 **실제 갓길선(노란 테이프)을 카메라로 보고** 우측으로 붙어 정차 (MRM 구현, R157 재출발 금지 래치 포함)
  - 🚧 **전방 장애물** → Hailo NPU + 라이다 퓨전으로 감지해 즉시 정지(동작 확인). 옆 차로로 피해 계속 달리는 회피주행은 이번 범위 밖으로, 향후 과제로 남겨둔다
  - ⚙️ **스택 다운** → `/cmd_vel`이 0.5초 끊기면 **차체 드라이버가 스스로 모터 정지** (구현 완료)
- **결과** — 실외 트랙에서 **모방학습 기반 자율주행**과 **OpenCV 규칙기반 차선 추종**을 둘 다 실제 소형 차체에서 end-to-end로 검증했고, **Hailo-8 NPU + 라이다 퓨전 장애물 감지 → 정지** 동작을 확인했다.

`/cmd_vel` 게이트는 **전방 장애물(정지) → 운전자 이상(갓길 대피) → 정상 주행** 순으로 가장 위급한 상황을 먼저 처리하고, 그 아래에 스택이 죽어도 동작하는 차체 드라이버 워치독을 이중으로 둔다.

---

## 🛠️ 기술 스택

**Robotics / 미들웨어**

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white)
![colcon](https://img.shields.io/badge/colcon-build-2E7D32)

**언어**

![Python](https://img.shields.io/badge/Python-3776AB?logo=python&logoColor=white)
![C++](https://img.shields.io/badge/C++-00599C?logo=cplusplus&logoColor=white)

**비전 / AI**

![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?logo=opencv&logoColor=white)
![Hailo-8](https://img.shields.io/badge/Hailo--8-NPU-FF6F00)
![YOLOv8](https://img.shields.io/badge/YOLOv8n-.hef-00A67E)
![PyTorch](https://img.shields.io/badge/PyTorch-EE4C2C?logo=pytorch&logoColor=white)
![ONNX](https://img.shields.io/badge/ONNX-005CED?logo=onnx&logoColor=white)
![MediaPipe](https://img.shields.io/badge/MediaPipe-0097A7?logo=google&logoColor=white)

**하드웨어**

![Raspberry Pi 5](https://img.shields.io/badge/Raspberry_Pi_5-A22846?logo=raspberrypi&logoColor=white)
![Camera Module 3](https://img.shields.io/badge/Camera_Module_3-imx708_(CSI)-6A1B9A)
![YDLIDAR X4](https://img.shields.io/badge/YDLIDAR-X4-1565C0)

| 분류 | 사용 기술 |
|---|---|
| 미들웨어 | ROS 2 Jazzy, colcon |
| 제어 · 인지 · 통신 노드 | Python |
| 차체 드라이버 (모터 · IMU · LiDAR) | C++ |
| 차선 인식 (규칙기반) | OpenCV (HSV 마스크 + Hough 변환 + P/헤딩 조향) |
| 자율주행 (모방학습) | PyTorch(학습) → ONNX → OpenCV DNN(Pi 추론), PilotNet 경량판 |
| 운전자 상태 감시 | MediaPipe FaceMesh (EAR 기반 눈감김 판정), 노트북에서 실행 |
| 객체 인식 | Hailo-8 NPU, YOLOv8n (`.hef`) |
| 센서 | Camera Module 3 (imx708/CSI), YDLIDAR X4 (360°, 0.12~10m) |

---

## 🏗️ 전체 시스템 아키텍처

```mermaid
%%{init: {'flowchart': {'nodeSpacing': 90, 'rankSpacing': 80}, 'themeVariables': {'fontSize': '22px'}}}%%
flowchart LR
    CAM["카메라"] --> OBJ["장애물 인식"]
    CAM --> DRIVE["주행 제어"]
    LIDAR["라이다"] --> DRIVE
    LIDAR --> GATE
    OBJ --> GATE
    DRIVE --> GATE
    BIO["노트북 웹캠<br/>운전자 감지"] -.-> GATE

    GATE{"안전 게이트"} -->|"cmd_vel"| CAR["차체 · 모터<br/>워치독 0.5s"]

    style GATE fill:#2b6cb0,stroke:#1a365d,color:#ffffff
```

> 평소엔 모방학습(`bc_follower`), 운전자 이상 시엔 규칙기반(`lane_follower`)이 **주행 제어**를 맡습니다. 둘 다 같이 떠 있지만 `driving_state`를 보고 한쪽만 `/cmd_vel_raw`를 발행해 서로 싸우지 않습니다. 장애물은 Hailo+라이다 퓨전으로 **주행 제어를 거치지 않고 안전 게이트가 즉시 정지**시킵니다.

### 판단 로직 (decision_maker)

```mermaid
flowchart LR
    A{"전방 장애물?<br/>Hailo + 라이다 퓨전<br/>(카메라 끊겨도 정지)"} -- 예 --> E["EMERGENCY_BRAKE<br/>즉시 정지"]
    A -- 아니오 --> B{"운전자 이상?<br/>(bio_anomaly, 래치)"}
    B -- 예 --> M["MRM_PULL_OVER<br/>우측 차로 감지 영역(UN R79) 확인<br/>(비었으면 우측 노란 갓길선 실시간 추종)"]
    B -- 아니오 --> N["NORMAL<br/>/cmd_vel_raw 통과<br/>(timeout 시 정지)"]
```

> 📂 패키지 구성 · 토픽 계약 · 빌드/실행 상세는 **[`docs/ARCHITECTURE.md`](./docs/ARCHITECTURE.md)** 에 정리되어 있습니다.

---

## 📚 참고문헌

- **NTREX STELLA_N5_ROS2** — 차체 베이스 플랫폼 · https://github.com/ntrexlab/STELLA_N5_ROS2 (출처/변경 이력: [`NOTICE.md`](./NOTICE.md))
- **Hailo hailo-rpi5-examples** — Hailo-8 NPU 추론 예제 · https://github.com/hailo-ai/hailo-rpi5-examples
- **camera_ros** — Raspberry Pi CSI 카메라 ROS 2 드라이버 · https://github.com/christianrauch/camera_ros
- **YOLOv8 (Ultralytics)** — 객체 인식 모델 · https://github.com/ultralytics/ultralytics
- **ROS 2 Jazzy 공식 문서** · https://docs.ros.org/en/jazzy/

### 안전 규격 · 선행기술

MRM(최소위험동작)과 운전자 상태 감시(DMS) 설계의 근거 문헌.

- **KR 공개특허 10-2024-0073259** — 「자율주행을 위한 MRM(최소위험동작) 장치와 방법 및 MRM 모드 결정 방법」, 한국전자통신연구원(ETRI), 2024.05.27 공개 (출원 10-2022-0154383)
  → MRM 6모드 정의와 모드 결정 플로우(청구항 13~16). 본 프로젝트는 이 중 **비상정차·직진정차·자차로정차·우차로정차** 4개를 구현.
- **UN Regulation No. 157 (ALKS)** — 최소위험조작의 규제상 정의, 정차 후 수동 입력 전 재출발 금지 · https://unece.org/sites/default/files/2025-06/R157r1e.pdf
- **UN Regulation No. 79 (조향장치, 04 series Supplement 6)**, EU 관보 OJ L 2025/3 (2025.01.10) · http://data.europa.eu/eli/reg/2025/3/oj
  → 원문 대조 완료. 운전자 무응답 시 **차로 밖(갓길)으로 차선변경해 정지**하는 RMF(Risk Mitigation Function)가 본 프로젝트의 우차로정차와 같은 기능이라 차선변경 판정을 이 조항에 맞췄다.
  - §2.3.4.5 RMF 정의 / §5.1.6.3.9.1 측방·후방 감지 능력이 있을 때만 차선변경 허용 / §5.1.6.3.9.2 위험 없이 못 가면 현재 차로 안에서 정지 → **자차로정차**
  - §5.6.4.8.2 감지 영역 그림: 옆 차로를 따라 뒤로 뻗은 직사각형, 측면 S_sensor,side = 6 m / §5.6.4.8.1 후방 감지거리 S_rear ≥ 55 m → 1/10 축소해 **폭 0.6 m × 길이 5.5 m** (`scan_sectors.right_lane_zone`)
  - §5.6.4.8.4 센서가 가려지면(blindness) 차선변경 금지 → `/scan` 끊기면 자차로정차
  - §5.6.4.7 임계거리 S_critical = (v_rear − v)·t_B + (v_rear − v)²/(2a) + v·t_G (a = 3 m/s², t_B = 0.4 s, t_G = 1 s) — 뒤차 속도가 필요한데 라이다 한 장으로는 알 수 없어, 영역 안에 **무엇이든 있으면** 차선변경을 포기하는 보수적 판정으로 대체
- **ISO 17387:2008, Lane change decision aid systems (LCDAS)** · https://www.iso.org/standard/43654.html
  → §2.4–2.5 인접 구역(adjacent zone)·후방 구역(rear zone)을 **차선 표시가 아니라 자차 기준 좌표로** 정의 — 감지 영역을 카메라 차선 인식과 무관하게 라이다 좌표로 잡은 근거. (구역 치수가 있는 §4.2는 유료 본문이라 미확인)
- **EU 2021/1341 (DDAW)** — 졸음 경고 의무 기준(KSS 8 이상) · https://eur-lex.europa.eu/legal-content/EN/TXT/HTML/?uri=CELEX:32021R1341
- **Euro NCAP, "Safe Driving — Driver Engagement" Protocol, Version 1.1, October 2025**, §1.3.3–1.3.5 · https://cdn.euroncap.com/cars/assets/euro_ncap_protocol_safe_driving_driver_engagement_v11_a30e874152.pdf
  → 원문 대조 완료. Microsleep(1~2초)·Sleep(≥3초)·Unresponsive(≥6초) 판정 시간이 `driver_monitor/drowsy_v5.py`의 상수(`MICROSLEEP_SEC`/`SLEEP_SEC`/`UNRESPONSIVE_SEC`) 그대로다.
- **PERCLOS** (Wierwille et al., NHTSA) — 눈 감김 비율 기반 졸음 지표(P80) · https://rosap.ntl.bts.gov/view/dot/113

### 모방학습(behavior cloning)

- **Bojarski, M. et al., "End to End Learning for Self-Driving Cars"**, NVIDIA, 2016 · https://arxiv.org/abs/1604.07316
  → `PilotNet` 원 논문. `scripts/train_bc.py`의 CNN이 이 구조를 경량화한 것.

### 차체 워치독(cmd_vel 타임아웃) — 근거 및 실사례

`stella_md`의 0.5초 cmd_vel 워치독은 "액추에이터는 소프트웨어(상위 안전 게이트)에만 안전을 의존해서는 안 된다"는 원칙을 따른 것이다 — 데드맨 스위치(dead man's switch) 패턴.

- **ISO 13849-1:2023** — 「Safety of machinery — Safety-related parts of control systems — Part 1: General principles for design」. 기계류 제어시스템 안전 부품 표준, Fail-safe 동작·이중화 설계를 요구 · https://www.iso.org/standard/73481.html
- **ISO 3691-4** — 무인운반차(AGV)의 안전 요구사항을 정의하는 규격
- **iRobot Create3 공식 문서 디스커션** — 실제 판매되는 ROS 2 로봇의 `/cmd_vel` 타임아웃이 **0.5초**로 본 프로젝트와 동일 · https://github.com/iRobotEducation/create3_docs/discussions/516
- **"stop my robot if /cmd_vel doesn't receive a message within a certain time period"** — ROS Answers, cmd_vel 워치독의 ROS 생태계 표준 구현 논의 · https://answers.ros.org/question/292512/
- **"Dead man's switch explained: A must for safe automation"**, Standard Bots · https://standardbots.com/blog/dead-mans-switch
- **"Safety for AGV/AMR" 백서**, MESCO Engineering, 2023 · https://mesco-engineering.com/wp-content/uploads/2023/06/mesco-whitepaper-design-package-safe-agv_230516.pdf
