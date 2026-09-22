<!-- markdownlint-disable MD033 MD041 -->
# 운전자 상태 감시(DMS) 구현 스펙

구현 담당: 정수영 · 코드: [`driver_monitor/drowsy_v5.py`](../driver_monitor/drowsy_v5.py) · 실행: 노트북(윈도우, ROS 없음)
출력 계약: [`docs/DRIVER_SIGNAL_CONTRACT.md`](DRIVER_SIGNAL_CONTRACT.md) (UDP 5005 → Pi `sensor_bridge_node`)

> **구현 현황 (drowsy_v5, 2026-09-21)** — 이 문서는 설계 스펙이고, 실제 구현은 아래처럼 단순화했다.
> - 구현: 개인 캘리브레이션(8초, EAR 80퍼센타일 기준 비율), 히스테리시스, PERCLOS(60초), 눈감김 분류(깜빡임/마이크로슬립/수면/무반응),
>   이벤트 채널 `MICROSLEEP(1s)` → `SLEEP(3s)` → `UNRESPONSIVE(6s)`, 졸음 채널(PERCLOS·마이크로슬립 빈도, 경고만), 얼굴 소실 누적, 경고음, CSV 로그.
> (SLEEP=3s·UNRESPONSIVE=6s는 Euro NCAP Safe Driving Driver Engagement 프로토콜의 asleep/unresponsive 기준 그대로다.)
> - **`bio_anomaly=True` 조건 = `SLEEP`(눈감김 또는 얼굴 소실 3초 이상)**, C 키로만 해제.
> - 미구현: KSS 추정, head pitch(고개 떨굼 각도), 개안 3초 자동 해제(C 키 수동 해제로 대체).
>   고개 떨굼은 얼굴 소실 경로로 일부 잡힌다.

이 문서는 **"자동차 업계가 실제로 쓰는 방식"** 을 축소 구현하기 위한 스펙이다.
흔한 오픈소스 졸음감지(고정 임계값 + 연속 프레임 카운트)와 무엇이 다른지가 이 프로젝트의 차별점이므로,
아래 규격 근거와 §2 비교표는 발표자료에 그대로 쓸 수 있도록 정리했다.

---

## 1. 근거 규격 (왜 이렇게 만드는가)

| 규격 | 내용 | 우리에게 주는 요구사항 |
|---|---|---|
| **EU 2021/1341 (DDAW)** | 졸음 수준이 **KSS 8 이상**이면 경고해야 한다(7에서 경고하는 것도 허용). 평가는 **KSS 또는 이와 동등함이 입증된 방법**으로 한다. | 출력이 "졸림/안졸림" 2값이 아니라 **KSS 등급**이어야 한다 |
| **PERCLOS P80** (Wierwille 1994, NHTSA 채택) | 단위시간(보통 1분) 중 **눈이 80% 이상 감겨 있던 시간의 비율**. 깜빡임이 아니라 졸음성 눈꺼풀 처짐(droop)을 본다. | 졸음 판정의 **주 지표**. 80%는 개폐 정도이지 EAR 절대값이 아니다 |
| **UN R157 (ALKS)** | 운전자가 인계 요구에 응답하지 않으면 **최소위험조작(MRM)** 으로 차량을 안전 정지 | 무반응 상태를 **졸음과 별도 단계**로 정의해야 한다 |
| **Euro NCAP 2026** | 최고 등급에 **연속 눈·머리 추적** 요구, 운전자 무반응 시 안전 정지에 가점 | 눈만이 아니라 **머리 자세**도 추적 |

> 용어: KSS = Karolinska Sleepiness Scale, 1(완전 각성)~9(싸워야 깨어있음)의 9단계 주관적 졸음 척도.

---

## 2. 흔한 구현 vs 업계 방식 ★발표 슬라이드용

| 항목 | 흔한 오픈소스 구현 | 업계/규격 방식 (= 우리가 할 것) |
|---|---|---|
| 눈 감김 판정 | `EAR < 0.25` **고정 임계값** | **운전자별 baseline 캘리브레이션** 후 **개폐율 80% 기준**(P80 정의 그대로) |
| 주 지표 | 연속 프레임 수 | **PERCLOS**(슬라이딩 윈도우) + 깜빡임 지속시간 + 깜빡임 빈도 |
| 출력 | 졸림 / 안졸림 | **KSS 등급 추정**(규격이 요구하는 척도) |
| 상태 전이 | 임계값 넘으면 즉시 | **히스테리시스 + 단계적 경고 에스컬레이션** |
| 머리 | 안 봄 | **head pose(pitch/yaw/roll)** — 고개 떨굼 = 급성 이상 신호 |
| 졸음 vs 무반응 | 구분 없음 | **분리**(DDAW 경고 단계 ≠ MRM 발동 단계) |
| 카메라 | RGB 웹캠 | **NIR 940nm + IR 조명** ← ❌ 우리는 불가(§5) |

고정 임계값이 왜 틀렸나: EAR 절대값은 **사람마다**(눈 크기, 쌍꺼풀), **거리·각도마다** 다르다.
P80의 "80%"는 EAR 0.2 같은 절대값이 아니라 **그 사람의 완전 개안 대비 80% 닫힘**을 뜻한다.
그래서 실제 시스템은 주행 시작 시 개인 기준값을 잡는다. 이 캘리브레이션 유무가 아마추어/프로의 갈림길이다.

---

## 3. 파이프라인

```
웹캠 30fps
  ▼
① 얼굴 랜드마크 (MediaPipe FaceMesh, 468점)
  ▼
② 개인 캘리브레이션 (주행 시작 시 1회)  ──▶ EAR_open, EAR_closed
  ▼
③ 프레임별 개폐율  closure = (EAR_open - EAR) / (EAR_open - EAR_closed)   [0~1]
  ▼
④ 지표 집계 : PERCLOS(60s 윈도우) · 깜빡임 지속시간 · 깜빡임 빈도
              최장 연속 폐안(microsleep) · head pitch · 얼굴 소실 시간
  ▼
⑤ KSS 추정 (규칙 기반 매핑)
  ▼
⑥ 상태머신 + 히스테리시스 : ALERT → DROWSY → SEVERE → INCAPACITATED
  ▼
⑦ /sensors/bio_anomaly (Bool, 2Hz+, latch)  ──▶ Pi decision_maker ──▶ MRM
```

### ② 개인 캘리브레이션 (필수 — 이게 핵심)

주행 시작 시 6초짜리 절차. 화면에 안내 문구를 띄우고 진행한다.

```
"눈을 뜨고 정면을 봐주세요" 3초  → EAR 중앙값 = EAR_open
"눈을 감아주세요"          3초  → EAR 중앙값 = EAR_closed
```

- 두 값의 차이가 너무 작으면(예: `EAR_open - EAR_closed < 0.08`) 캘리브레이션 실패로 보고 재시도.
- 캘리브레이션 값은 파일로 저장해 재실행 시 재사용 가능하게(시연 중 반복 실행 대비).
- 캘리브레이션 없이 돌릴 때를 위한 fallback 고정값(`EAR_open=0.30, EAR_closed=0.10`)도 두되, **화면에 "UNCALIBRATED" 경고를 띄운다.**

### ③ 개폐율

```python
closure = (ear_open - ear) / (ear_open - ear_closed)   # 0=완전히 뜸, 1=완전히 감음
closed  = closure >= 0.80                              # P80 정의
```
좌/우 눈 EAR의 평균을 쓴다. 한쪽만 보이면(고개 돌림) 보이는 쪽만.

### ④ 지표

| 지표 | 정의 | 용도 |
|---|---|---|
| **PERCLOS** | 최근 `perclos_window`(기본 60s) 중 `closed`인 프레임 비율 | 졸음 주 지표 |
| **blink duration** | `closed` 구간의 지속시간 (정상 0.1~0.4s, 졸릴수록 길어짐) | 졸음 보조 |
| **blink rate** | 분당 깜빡임 수 | 졸음 보조 |
| **microsleep** | `closed`가 끊기지 않고 이어진 최장 시간 | **무반응 판정** |
| **head pitch** | `solvePnP`로 구한 머리 상하 각도. 기준 대비 아래로 N도 이상 | **무반응 판정**(고개 떨굼) |
| **face lost** | 얼굴이 검출되지 않은 연속 시간 | **무반응 판정**(완전히 쓰러짐) |

### ⑤ KSS 추정

규칙 기반으로 충분하다(문헌의 PERCLOS↔KSS 경향을 단조 매핑). 값은 트랙/조명에 맞춰 튜닝.

| 조건 | 추정 KSS |
|---|---|
| PERCLOS < 0.08 이고 평균 blink duration < 0.3s | 1~4 (각성) |
| PERCLOS 0.08~0.15 | 5~6 |
| PERCLOS 0.15~0.25 **또는** blink duration > 0.4s | **7** (DDAW 경고 허용선) |
| PERCLOS > 0.25 **또는** blink duration > 0.5s | **8 이상** (DDAW 경고 **의무선**) |

> ⚠️ 이 매핑은 **인간 피험자 검증을 거치지 않은 추정치**다. 규정은 KSS 동등성 입증에 피험자 실험을 요구한다(§5).
> 보고서·발표에서 "KSS 추정(estimated)"이라고 반드시 명시할 것.

### ⑥ 상태머신

```
                    ┌──────────────────────────────────────────┐
                    │                                          │ (해제 조건 성립)
                    ▼                                          │
   ALERT ──KSS≥7──▶ DROWSY ──KSS≥8──▶ SEVERE ──────────────────┤
  (정상)           (1차 경고)        (2차 경고, DDAW 의무)      │
                                          │                    │
                            microsleep ≥ 2.0s                  │
                         또는 head pitch 떨굼 ≥ 2.0s            │
                         또는 face lost ≥ 2.0s                  │
                                          ▼                    │
                                  INCAPACITATED ───────────────┘
                                  bio_anomaly = True (LATCH)
```

- **히스테리시스**: 상승 조건과 하강 조건을 다르게 둔다. 예) KSS 8로 SEVERE 진입, 6 이하가 **5초 지속**되어야 DROWSY로 하강.
- **INCAPACITATED는 latch.** 해제는 "눈 뜬 상태(closure < 0.5)가 **3초 연속** 유지" 일 때만.
  한 프레임 튐으로 해제되면 **차가 갓길에서 다시 출발한다.**
- 경고 단계(DROWSY/SEVERE)에서는 `bio_anomaly=False`를 유지한다 — 졸린다고 차를 세우지는 않는다.
  이 분리가 DDAW(경고)와 MRM(정지)의 규격상 구분을 그대로 반영한 것이다.

### ⑦ 출력

- `/sensors/bio_anomaly` (std_msgs/Bool) — `INCAPACITATED`일 때만 `True`. 2Hz 이상 상시 발행.
- 디버그 창(시연 영상용, 필수) — 얼굴 랜드마크 · `closure` · `PERCLOS` · `KSS` · 현재 상태 ·
  **Pi에서 받은 `/control/driving_state`**. 마지막 항목이 있어야 "눈 감음 → 차가 갓길로"가 한 화면에서 증명된다.
- CSV 로그(권장) — `timestamp, ear, closure, perclos, blink_dur, kss, state`.
  보고서 그래프(시간축 PERCLOS/KSS 추이)를 이 파일로 바로 그릴 수 있다.

---

## 4. 파라미터

| 이름 | 기본값 | 의미 |
|---|---|---|
| `perclos_window` | 60.0 | PERCLOS 슬라이딩 윈도우(초). 문헌은 1~20분, 시연 편의상 60초 |
| `closure_threshold` | 0.80 | P80 정의값. 바꾸지 말 것 |
| `kss_warn` / `kss_severe` | 7 / 8 | DDAW 경고선 |
| `microsleep_sec` | 2.0 | 무반응 판정 연속 폐안 시간 |
| `head_drop_deg` | 20.0 | 기준 대비 고개 숙임 각도 |
| `head_drop_sec` | 2.0 | 고개 떨굼 지속시간 |
| `face_lost_sec` | 2.0 | 얼굴 소실 지속시간 |
| `recover_sec` | 3.0 | INCAPACITATED 해제에 필요한 개안 지속시간 |

---

## 5. 우리가 못 하는 것 (보고서에 명시할 한계)

정직하게 적는 편이 질의응답에서 유리하다. 축소 모델의 한계를 아는 것 자체가 평가 대상이다.

1. **NIR(근적외선) 카메라 없음** — 실차 DMS는 940nm NIR 카메라 + IR 조명을 써서 야간·역광·선글라스 상황에서도 동공과 눈꺼풀을 본다. RGB 웹캠은 이게 불가능하므로 **조명이 있는 실내 환경으로 한정**된다.
2. **KSS 동등성 미검증** — 규정은 인간 피험자 주행 실험으로 KSS 동등성을 입증하라고 요구한다. 우리 매핑은 문헌 경향을 따른 **추정치**다.
3. **간접 지표 미사용** — 실차는 조향 패턴·차선 유지 성능 같은 주행 행태도 함께 본다(다만 Euro NCAP 2026부터 간접 단독으로는 만점 불가). 우리는 직접 관측만 쓴다.
4. **주의분산(ADDW) 미구현** — EU 2023/2590은 시선 이탈 기반 주의분산 경고를 따로 규정한다. 이번 시나리오는 **무반응(급성 이상)** 이므로 범위에서 제외했다.
5. **탑승 불가** — 1:10 축소 차량이라 운전석 카메라를 차에 달 수 없어, 노트북 웹캠으로 운전석 DMS를 대체했다.

---

## 6. 검증 (구현 후 이것만 돌려보면 됨)

| # | 시나리오 | 기대 |
|---|---|---|
| 1 | 캘리브레이션 후 정상 상태 30초 | PERCLOS < 0.08, KSS 1~4, `ALERT`, `bio_anomaly=False` 연속 발행 |
| 2 | 의도적으로 천천히 깜빡이기 30초 | PERCLOS·blink duration 상승, KSS 7↑, `DROWSY`/`SEVERE`, **`bio_anomaly`는 여전히 False** |
| 3 | 눈 감고 3초 | `INCAPACITATED` 진입, `bio_anomaly=True` |
| 4 | 3번 직후 눈 뜨고 1초 | **여전히 True** (latch 확인 — 여기서 False로 풀리면 버그) |
| 5 | 눈 뜨고 3초 유지 | `ALERT` 복귀, `False` |
| 6 | 고개만 푹 숙이기 3초 | `INCAPACITATED` (눈이 보이지 않아도 잡혀야 함) |
| 7 | 카메라 가리기 3초 | `INCAPACITATED` (face lost 경로) |

4번이 제일 중요하다 — 실패하면 차가 갓길에서 재출발한다.

---

## 7. 참고문헌

- [EU 2021/1341 (DDAW) 원문 — EUR-Lex](https://eur-lex.europa.eu/legal-content/EN/TXT/HTML/?uri=CELEX:32021R1341)
- [PERCLOS: A Valid Psychophysiological Measure of Alertness (NHTSA/FHWA)](https://rosap.ntl.bts.gov/view/dot/113)
- [Drowsiness Detection System Based on PERCLOS and Facial Physiological Signal (Sensors, 2022)](https://www.mdpi.com/1424-8220/22/14/5380)
- [UN Regulation No. 157 (ALKS) 원문 — UNECE](https://unece.org/sites/default/files/2025-06/R157r1e.pdf)
- [Euro NCAP, "Safe Driving — Driver Engagement" Protocol, Version 1.1, October 2025, §1.3.3–1.3.5](https://cdn.euroncap.com/cars/assets/euro_ncap_protocol_safe_driving_driver_engagement_v11_a30e874152.pdf) — Microsleep(1~2초)·Sleep(≥3초)·Unresponsive(≥6초) 원문 대조 완료(2026-09-22)
- [Seeing Machines — DDAW 시스템 해설](https://seeingmachines.com/understanding-driver-drowsiness-and-attention-warning-ddaw-systems/)
- [Driver Drowsiness Detection Using MediaPipe (LearnOpenCV, 구현 참고)](https://learnopencv.com/driver-drowsiness-detection-using-mediapipe-in-python/)
