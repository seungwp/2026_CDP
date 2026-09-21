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
- **3가지 안전 시나리오**
  - 🫀 **운전자 생체 이상** → 감속하며 우측 갓길로 이동 후 정차 (MRM 로직 구현)
  - 🚧 **전방 장애물** → Hailo NPU로 감지해 정지(동작 확인). 단순 정지를 넘어 여유 공간으로 **회피 주행** 하도록 개발 중
  - ⚙️ **스택 다운** → `/cmd_vel`이 0.5초 끊기면 **차체 드라이버가 스스로 모터 정지** (구현 완료)
- **결과** — 실외 트랙에서 **OpenCV 기반 차선 추종 자율주행** 을 실제 소형 차체에서 end-to-end로 검증했고, **Hailo-8 NPU 장애물 감지 → 정지** 동작을 확인했다.

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

**하드웨어**

![Raspberry Pi 5](https://img.shields.io/badge/Raspberry_Pi_5-A22846?logo=raspberrypi&logoColor=white)
![Camera Module 3](https://img.shields.io/badge/Camera_Module_3-imx708_(CSI)-6A1B9A)
![YDLIDAR X4](https://img.shields.io/badge/YDLIDAR-X4-1565C0)

| 분류 | 사용 기술 |
|---|---|
| 미들웨어 | ROS 2 Jazzy, colcon |
| 제어 · 인지 · 통신 노드 | Python |
| 차체 드라이버 (모터 · IMU · LiDAR) | C++ |
| 차선 인식 | OpenCV (HSV 마스크 + P/D 조향) |
| 객체 인식 | Hailo-8 NPU, YOLOv8n (`.hef`) |
| 센서 | Camera Module 3 (imx708/CSI), YDLIDAR X4 (360°, 0.12~10m) |

---

## 🏗️ 전체 시스템 아키텍처

```mermaid
%%{init: {'flowchart': {'nodeSpacing': 110, 'rankSpacing': 95}, 'themeVariables': {'fontSize': '24px'}}}%%
flowchart LR
    CAM["📷 카메라"] --> LANE["차선 인식"]
    CAM --> OBJ["장애물 인식<br/>(Hailo NPU)"]
    LANE --> DRIVE["차선 추종 주행"]

    DRIVE -->|주행 명령| GATE
    OBJ -->|장애물| GATE
    BIO["🩺 운전자 생체신호"] -->|이상 신호| GATE

    LIDAR["📡 라이다"] -->|후방·측방 여유| DRIVE
    GATE["🧭 안전 게이트<br/>decision_maker"] -->|"/cmd_vel"| CAR["🚗 차체 · 모터<br/>워치독 0.5s"]

    style GATE fill:#2b6cb0,stroke:#1a365d,color:#ffffff
```

> 모든 주행 명령은 **안전 게이트** 하나를 거쳐 바퀴로 나가고, 스택이 멈추면 **차체 드라이버**가 직접 멈춥니다.

### 판단 로직 (decision_maker)

```mermaid
flowchart LR
    A{"전방 장애물?<br/>Hailo + 라이다 퓨전<br/>(카메라 끊겨도 정지)"} -- 예 --> E["EMERGENCY_BRAKE<br/>즉시 정지"]
    A -- 아니오 --> B{"운전자 이상?<br/>(bio_anomaly, 래치)"}
    B -- 예 --> M["MRM_PULL_OVER<br/>라이다로 모드 결정 후 대피"]
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
- **EU 2021/1341 (DDAW)** — 졸음 경고 의무 기준(KSS 8 이상) · https://eur-lex.europa.eu/legal-content/EN/TXT/HTML/?uri=CELEX:32021R1341
- **Euro NCAP 2026 프로토콜** — 연속 눈·머리 추적, 무반응 운전자 대응 가점 · https://www.euroncap.com/press-media/euro-ncap-announces-2026-protocol-changes-to-tackle-modern-driving-risks/
- **PERCLOS** (Wierwille et al., NHTSA) — 눈 감김 비율 기반 졸음 지표(P80) · https://rosap.ntl.bts.gov/view/dot/113
