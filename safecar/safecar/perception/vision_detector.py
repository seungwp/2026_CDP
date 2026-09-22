import cv2
import numpy as np


class VisionDetector:
    """카메라 프레임에서 차선을 인식해 차로 중심 대비 횡방향 오프셋을 계산한다. (인지부, ROS 비의존 순수 로직)

    장애물 인식은 이 클래스가 아니라 Hailo NPU 노드(stella_hailo_rpi5_ros2_examples)가 담당한다.

    오프셋 규약: -1.0 ~ +1.0 정규화 값.
    + 는 차로 중심이 화면 중앙보다 오른쪽에 있음(차가 왼쪽으로 치우침 → 우조향 필요),
    - 는 그 반대. 차선을 하나도 못 찾으면 None.
    """

    # 튜닝 파라미터 — 테스트 트랙(테이프 색·조명·카메라 각도)에 맞춰 조정할 것
    # 2026-07-08 실측 기준: 카메라가 거의 수평이라 테이프가 화면 중간(0.45~0.7)에 보이고
    # 근접 차선은 좌우로 화면을 벗어난다. 카메라를 아래로 숙여 달면 이 값들 재튜닝 필요.
    ROI_TOP = 0.35        # 화면 높이의 이 비율 지점부터 아래에서만 차선 탐색.
                          # 점선 차선은 조각 사이 공백이 있어, 다음 조각까지 보이도록
                          # 넓게 잡아야 공백 구간에서 차선을 놓치지 않는다.
    Y_EVAL = 0.6          # 오프셋을 재는 기준 행(높이 비율). 작을수록 더 멀리 본다.
                          # 멀리 볼수록 곡선 진입을 미리 감지해 선제 조향이 되지만,
                          # 같은 횡오차가 픽셀로는 작게 잡히므로 steer_gain을 같이 올려야 한다.
    MIN_ABS_SLOPE = 0.8   # 이보다 완만한 선분은 차선으로 안 봄. 실외 트랙 실측:
                          # 실제 차선은 기울기 1.5~2.5, 노면 얼룩/옆차로 잔상은 1.0 이하.
    HALF_LANE_PX = 340    # (양쪽 차선 모드 전용) 한쪽만 보일 때 가정하는 차로 반폭(픽셀)
    USE_WHITE = True      # 흰색 마스크. 실외 아스팔트의 흰 차선용.
                          # 실내 광택 바닥에서는 조명 반사가 가짜 차선이 되므로 꺼야 한다.
    USE_YELLOW = False    # 노란 마스크. 현재 실외 트랙은 흰 차선뿐이라 끔.
                          # 실내 노란 테이프 트랙으로 돌아가면 True + USE_WHITE=False.
    # 갓길(우측 노란선) 전용 기울기 문턱. 화면 중앙의 흰 차선은 기울기 1.5~2.5로 거의
    # 세워져 보이지만, 카메라가 낮고 거의 수평이라 **옆으로 30~50cm 벗어난** 선은
    # 원근 때문에 훨씬 눕게 보인다 — 2026-09-22 실측: 갓길 테이프 10프레임 전부
    # 기울기 0.22~0.64 (MIN_ABS_SLOPE=0.8 기준으로는 전부 걸러져서 한 프레임도
    # 감지가 안 됐다). 그 범위보다 낮게 잡아 여유를 둔다.
    SHOULDER_MIN_ABS_SLOPE = 0.15
    # MAX_ABS_OFFSET(흰선 유실 판정용 0.7)도 갓길선엔 안 맞는다 — 갓길선은 처음 전환되는
    # 순간 "화면 가장자리에 있는 게 정상"이다(그래서 우측으로 붙어야 하는 것). 0.7로
    # 두면 딱 그 첫 프레임에 "너무 멀어서 유실"로 처리돼 조향 신호 자체가 안 나간다
    # (2026-09-22 실측: 처음 검출 offset이 0.76이었음). 1.0(=클립 상한, 사실상 무제한)로 둔다.
    SHOULDER_MAX_ABS_OFFSET = 1.0
    # 노란색 판정 기준(채도/밝기 하한). 기존 70/70은 주간 실측값인데, 2026-09-22 야간
    # 트랙 스크린샷을 직접 샘플링해보니 같은 노란 테이프도 두 방향으로 이 범위를 벗어났다:
    #   - 먼 쪽(어두움): 채도는 150~200으로 충분한데 밝기가 53~66 (V<70이라 전부 컷됨)
    #   - 가까운 쪽(가로등/젖은 노면 반사로 뜸): 밝기는 118~191인데 채도가 8~68까지 떨어짐
    #     (S<70이라 전부 컷됨) — 그래서 화면에 선이 뻔히 보여도 NO LANE이 떴다.
    # 주간 트랙은 70/70을 그대로 쓰고, 갓길(우차로정차) 모드만 아래 값으로 낮춘다.
    YELLOW_S_MIN = 70
    YELLOW_V_MIN = 70
    SHOULDER_YELLOW_S_MIN = 25   # 야간 실측 최저 채도(8)까진 아니어도 대부분 잡도록 여유
    SHOULDER_YELLOW_V_MIN = 40   # 야간 실측 원거리 밝기(53) 구간을 살리면서 순수 아스팔트(<40)는 컷
    # 흰색 판정 기준. **직사광선 아래에서 제일 중요한 값이다** — 햇빛 받은 아스팔트가
    # V=200을 쉽게 넘어서 노면 전체가 차선으로 잡힌다(실측: ROI의 6%가 V≥200).
    # 흐린 날/실내로 조명이 바뀌면 다시 낮춰야 한다.
    WHITE_V_MIN = 200     # 이 밝기 이상만 흰색으로 본다.
                          # 실측: 230으로 올리면 햇빛 잡음은 줄지만 멀리 있는(어두운)
                          # 점선 조각이 먼저 잘려나간다. 실외 트랙 검증값은 200.
    WHITE_S_MAX = 40      # 이 채도 이하만 흰색으로 본다(무채색)

    # 단일선 추종 모드: 좌/우 차선을 짝지어 중심을 추정하지 않고, **선 하나를 직접 따라간다.**
    # 실외 트랙처럼 차로가 여러 개라 옆 차로 선까지 잡히는 상황에서 훨씬 안정적이다.
    # (양쪽 차선이 하나씩만 보이는 단순 트랙이면 False가 낫다)
    FOLLOW_SINGLE_LINE = True
    CLUSTER_PX = 60       # 같은 선으로 묶을 x 허용 범위(픽셀, y_eval 행 기준)
    # 추종 중인 선이 화면 중앙에서 이만큼 넘게 벗어나면 '차선 유실'로 본다.
    # 선을 제대로 따라가는 중이라면 이렇게까지 치우칠 수 없다 — 즉 엉뚱한 걸(노면 얼룩,
    # 옆 차로 선) 쫓고 있다는 뜻이다. 그대로 두면 _last_center가 잡음에 물린 채
    # 영영 안 풀리고, 차가 그 잡음을 따라 코스를 벗어난다(실측: 12.8m 폭주).
    # 유실로 처리하면 lane_follower가 정지시키고 기준도 초기화된다.
    MAX_ABS_OFFSET = 0.7

    # 추종 중인 선이 한 프레임에 이만큼 넘게 튀면 '같은 선'으로 인정하지 않는다.
    # 45Hz·0.15m/s에서 실제 차선은 프레임당 몇 픽셀만 움직인다. 200px씩 건너뛰는 건
    # 물리적으로 불가능하고, 그건 **옆 차선으로 갈아타는 중**이라는 뜻이다.
    # 점선의 조각 사이 공백에서는 옆 차선만 검출되므로, 이 게이트가 없으면 거기로 넘어간다.
    MAX_JUMP_PX = 45
    # 게이트 밖 후보만 있을 때 직전 값으로 버티는 프레임 수. 점선 공백을 넘기기 위한 것.
    # 이보다 길어지면 진짜로 놓친 것으로 보고 유실 처리한다(45Hz 기준 약 0.4초).
    MAX_COAST_FRAMES = 18

    def __init__(self):
        self._last_center = None  # 직전 프레임에서 따라가던 선의 x (선 바꿔타기 방지)
        self._coast = 0           # 게이트 밖 후보만 있어 직전 값으로 버틴 프레임 수
        self._last_heading = 0.0  # 추종 중인 선의 기울기(헤딩 오차)
        # 인스턴스 값으로 복사해둔다 — set_shoulder_mode()가 여기만 바꾸고
        # 클래스 상수(USE_WHITE/USE_YELLOW)는 "평소 설정값"으로 그대로 둔다.
        self.use_white = self.USE_WHITE
        self.use_yellow = self.USE_YELLOW
        self.min_abs_slope = self.MIN_ABS_SLOPE
        self.max_abs_offset = self.MAX_ABS_OFFSET
        self.yellow_s_min = self.YELLOW_S_MIN
        self.yellow_v_min = self.YELLOW_V_MIN
        print("[System] Vision: OpenCV 차선 인식 초기화 완료.")

    def set_shoulder_mode(self, active):
        """MRM 우차로정차 동안 흰 차선 대신 갓길(노란 테이프)을 보게 전환한다.

        새 추적 로직을 만들지 않고 기존 단일선 추종을 그대로 재사용한다 — 색과 기울기
        문턱만 바꾼다(SHOULDER_MIN_ABS_SLOPE, 이유는 그 상수 주석 참고).
        전환 순간 추종 상태를 리셋한다. 안 그러면 MAX_JUMP_PX 게이트가 '색이 바뀐 새
        선'을 옛 선과 다른 위치라며 몇 프레임 동안 거부한다(불필요한 지연).
        """
        if active:
            self.use_white, self.use_yellow = False, True
            self.min_abs_slope = self.SHOULDER_MIN_ABS_SLOPE
            self.max_abs_offset = self.SHOULDER_MAX_ABS_OFFSET
            self.yellow_s_min = self.SHOULDER_YELLOW_S_MIN
            self.yellow_v_min = self.SHOULDER_YELLOW_V_MIN
        else:
            self.use_white, self.use_yellow = self.USE_WHITE, self.USE_YELLOW
            self.min_abs_slope = self.MIN_ABS_SLOPE
            self.max_abs_offset = self.MAX_ABS_OFFSET
            self.yellow_s_min = self.YELLOW_S_MIN
            self.yellow_v_min = self.YELLOW_V_MIN
        self._last_center = None
        self._coast = 0
        self._last_heading = 0.0

    def process_frame(self, frame):
        """(디버그 프레임, 오프셋, 헤딩오차)를 반환한다. 차선을 못 찾으면 뒤 둘은 None.

        헤딩오차: 차선이 멀어지면서 좌우로 기우는 정도(+는 우측으로 휨).
        횡오차만 보는 P 제어는 곡선에서 구조적으로 밀리는데, 이 항이 있으면
        차가 옆으로 밀리기 **전에** 곡선 방향을 알 수 있다(Stanley 제어의 헤딩 항).
        """
        height, width = frame.shape[:2]

        # 1. 차선 색 마스크 (노란 테이프, 실측 기준으로 여유 있게)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        if self.use_yellow:
            mask = cv2.inRange(hsv, np.array([18, self.yellow_s_min, self.yellow_v_min]),
                               np.array([40, 255, 255]))
        if self.use_white:
            mask_white = cv2.inRange(hsv, np.array([0, 0, self.WHITE_V_MIN]),
                                     np.array([180, self.WHITE_S_MAX, 255]))
            mask = cv2.bitwise_or(mask, mask_white)

        # 2. 하단 ROI에서만 엣지/직선 검출
        edges = cv2.Canny(mask, 50, 150)
        roi_top = int(height * self.ROI_TOP)
        roi = np.zeros_like(edges)
        cv2.rectangle(roi, (0, roi_top), (width, height), 255, -1)
        masked_edges = cv2.bitwise_and(edges, roi)

        lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 50,
                                minLineLength=40, maxLineGap=120)

        debug = frame.copy()
        cv2.line(debug, (width // 2, roi_top), (width // 2, height), (255, 0, 0), 1)

        # 3. 각 선분을 기준 행(y_eval)까지 연장한 x좌표로 좌/우 차선 분류
        y_eval = int(height * self.Y_EVAL)
        xs, tilts = [], []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                dx, dy = x2 - x1, y2 - y1
                if dx == 0:
                    x_at = float(x1)  # 수직선
                else:
                    slope = dy / dx
                    if abs(slope) < self.min_abs_slope:
                        continue
                    x_at = x1 + (y_eval - y1) / slope
                # 근접 차선은 기준 행에서 화면 밖으로 나가는 게 정상(카메라가 낮아서).
                # 화면 폭의 ±1배까지는 유효한 차선으로 인정하고, 그 이상만 노이즈로 버린다.
                if not (-width <= x_at < 2 * width):
                    continue
                # 차선이 멀어지면서 좌우로 얼마나 기우는지 = 차량과 차선의 각도 차(헤딩 오차).
                # 화면에서 위쪽(y가 작은 쪽)이 더 먼 지점이다.
                if y1 <= y2:
                    far_x, far_y, near_x, near_y = x1, y1, x2, y2
                else:
                    far_x, far_y, near_x, near_y = x2, y2, x1, y1
                tilt = (far_x - near_x) / max(abs(far_y - near_y), 1.0)
                cv2.line(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
                xs.append(x_at)
                tilts.append(float(np.clip(tilt, -2.0, 2.0)))

        if not xs:
            cv2.putText(debug, "NO LANE", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            self._last_center = None  # 재획득 시 화면 중앙부터 다시 찾는다
            self._last_heading = 0.0
            return debug, None, None

        # 4. 따라갈 기준선 결정
        if self.FOLLOW_SINGLE_LINE:
            # 직전에 따라가던 선에 가장 가까운 것을 고른다. 기준이 없으면 화면 중앙에서 시작.
            # (매 프레임 '화면 중앙에 가장 가까운 선'을 고르면, 차가 밀릴 때 옆 차로 선으로
            #  갈아타 버린다 — 이걸 막는 게 _last_center의 역할)
            if self._last_center is None:
                # 재획득: 기준이 없으니 화면 중앙에서 가장 가까운 선을 잡는다.
                picked = min(xs, key=lambda x: abs(x - width / 2.0))
                in_gate = True
            else:
                picked = min(xs, key=lambda x: abs(x - self._last_center))
                in_gate = abs(picked - self._last_center) <= self.MAX_JUMP_PX

            if in_gate:
                idx = [i for i, x in enumerate(xs) if abs(x - picked) <= self.CLUSTER_PX]
                center = float(np.median([xs[i] for i in idx]))
                # 추종 중인 선을 이루는 선분들의 기울기 평균 = 헤딩 오차
                self._last_heading = float(np.median([tilts[i] for i in idx]))
                self._coast = 0
            else:
                # 게이트 밖 후보뿐 — 같은 선일 수 없다(점선 공백에서 옆 차선만 보이는 상황).
                # 갈아타지 말고 직전 값으로 버틴다.
                self._coast += 1
                if self._coast > self.MAX_COAST_FRAMES:
                    cv2.putText(debug, "LOST (no line in gate)", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    self._last_center = None
                    self._coast = 0
                    self._last_heading = 0.0
                    return debug, None, None
                center = self._last_center  # coast — 헤딩은 직전 값을 유지한다

            self._last_center = center
        else:
            # 양쪽 차선 모드: 좌/우로 나눠 중심을 추정 (한쪽만 보이면 반폭 가정)
            left_xs = [x for x in xs if x < width / 2]
            right_xs = [x for x in xs if x >= width / 2]
            if left_xs and right_xs:
                center = (np.median(left_xs) + np.median(right_xs)) / 2.0
            elif left_xs:
                center = np.median(left_xs) + self.HALF_LANE_PX
            else:
                center = np.median(right_xs) - self.HALF_LANE_PX

        offset = float(np.clip((center - width / 2) / (width / 2), -1.0, 1.0))

        if abs(offset) > self.max_abs_offset:
            cv2.putText(debug, f"LOST (off={offset:+.2f})", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.circle(debug, (int(center), y_eval), 6, (0, 0, 255), 2)
            self._last_center = None  # 잡음에 물린 기준을 풀어준다
            self._last_heading = 0.0
            return debug, None, None

        cv2.circle(debug, (int(center), y_eval), 6, (0, 0, 255), -1)
        cv2.putText(debug, f"off={offset:+.2f} head={self._last_heading:+.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return debug, offset, self._last_heading
