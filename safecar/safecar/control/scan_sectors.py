"""LaserScan에서 특정 방향 섹터의 최소거리를 뽑는다. (제어부, ROS 비의존 순수 로직)

갓길 대피(MRM) 전에 "뒤가 비었나 / 갓길 쪽에 설 자리가 있나"를 판단하는 데 쓴다.

각도 규약: ROS 표준(REP-103) 기준 전방 0°, 좌측 +90°, 우측 -90°, 후방 ±180°.
다만 라이다가 차체에 어떤 방향으로 장착됐는지, YDLIDAR의 `reversion` 설정이 어떤지에 따라
실제 대응이 달라지므로 **섹터 중심각은 반드시 실측으로 확인하고 파라미터로 조정할 것.**
(확인 방법: 차 우측에만 물체를 두고 어느 중심각에서 거리가 줄어드는지 본다.)
"""

import math


def sector_min(ranges, angle_min, angle_increment,
               center_deg, half_width_deg,
               range_min=0.0, range_max=float('inf')):
    """중심각 ±half_width 섹터 안의 최소 유효거리. 유효한 측정이 하나도 없으면 None.

    inf/nan(미반사)과 [range_min, range_max] 밖의 값은 버린다 — 라이다는 측정 실패를
    inf나 0으로 채우는데, 이걸 거리로 믿으면 '벽이 코앞에 있다'고 오판한다.
    """
    if not ranges or angle_increment == 0.0:
        return None

    center = math.radians(center_deg)
    half = math.radians(abs(half_width_deg))
    best = None

    for i, r in enumerate(ranges):
        if r is None or math.isnan(r) or math.isinf(r):
            continue
        if not (range_min <= r <= range_max):
            continue
        # 중심각과의 차이를 [-pi, pi]로 감아서 ±180° 경계(후방)를 제대로 처리한다.
        diff = (angle_min + i * angle_increment) - center
        diff = (diff + math.pi) % (2.0 * math.pi) - math.pi
        if abs(diff) <= half and (best is None or r < best):
            best = r

    return best


# --- 우측 차로 감지 영역 (차선변경 가능 여부) ---
# UN R79 §5.6.4.8.2(ACSF Category C 감지 영역 그림)를 1/10로 축소했다. 규정의 감지
# 영역은 부채꼴이 아니라 **옆 차로를 따라 뒤로 뻗은 직사각형**이다(자기 차로 뒤는
# 포함하지 않음). 옆으로 S_sensor,side = 6 m, 뒤로 S_rear ≥ 55 m(§5.6.4.8.1).
# 좌표는 라이다 기준 REP-103: x 전방+, y 좌측+ (우측은 y<0).
ZONE_SIDE_M = 0.6    # 6 m × 1/10
ZONE_REAR_M = 5.5    # 55 m × 1/10
# 차체 치수(라이다 중심 기준). 2026-09-22 실측: 폭 380 mm × 길이 440 mm.
# 라이다가 차체 정중앙에 있다고 가정해 절반씩 나눴다 — 앞뒤로 치우쳐 달려 있으면
# FRONT/REAR를 라이다 중심에서 실제로 잰 값으로 고칠 것.
VEHICLE_HALF_WIDTH_M = 0.19   # 라이다 중심 ~ 차체 우측면 (380/2)
VEHICLE_FRONT_M = 0.22        # 라이다 중심 ~ 차체 앞끝 (440/2)
VEHICLE_REAR_M = 0.22         # 라이다 중심 ~ 차체 뒤끝 (440/2)


def right_lane_zone(half_width=VEHICLE_HALF_WIDTH_M, front=VEHICLE_FRONT_M,
                    rear=VEHICLE_REAR_M, side=ZONE_SIDE_M, rear_len=ZONE_REAR_M):
    """우측 차로 감지 영역 → (x_min, x_max, y_min, y_max). 차체 옆면부터 옆으로 side,
    차체 앞끝부터 뒤끝 뒤로 rear_len까지."""
    return (-(rear + rear_len), front, -(half_width + side), -half_width)


def zone_nearest(ranges, angle_min, angle_increment, box,
                 range_min=0.0, range_max=float('inf')):
    """직사각형 box=(x_min, x_max, y_min, y_max) 안에서 라이다에 가장 가까운 점.
    → (거리, x, y), 영역 안에 점이 없으면 None. 무효값 처리는 sector_min과 같다."""
    x_min, x_max, y_min, y_max = box
    best = None
    for i, r in enumerate(ranges):
        if r is None or math.isnan(r) or math.isinf(r):
            continue
        if not (range_min <= r <= range_max):
            continue
        a = angle_min + i * angle_increment
        x, y = r * math.cos(a), r * math.sin(a)
        if x_min <= x <= x_max and y_min <= y <= y_max and (best is None or r < best[0]):
            best = (r, x, y)
    return best


def is_clear(ranges, angle_min, angle_increment,
             center_deg, half_width_deg, clear_dist,
             range_min=0.0, range_max=float('inf'), unknown_is_clear=True):
    """섹터가 clear_dist보다 멀리까지 비었는지. 측정값이 없으면 unknown_is_clear를 따른다."""
    d = sector_min(ranges, angle_min, angle_increment, center_deg, half_width_deg,
                   range_min, range_max)
    if d is None:
        return unknown_is_clear
    return d > clear_dist


def _self_check():
    n = 360
    angle_min = -math.pi
    inc = 2.0 * math.pi / n

    def idx(deg):  # 각도 → 인덱스
        return int(round((math.radians(deg) - angle_min) / inc)) % n

    ranges = [5.0] * n
    ranges[idx(90)] = 0.3     # 좌측에 가까운 물체
    ranges[idx(-178)] = 0.5   # 후방(경계 넘어감)
    ranges[idx(0)] = float('inf')   # 전방 미반사
    ranges[idx(1)] = float('nan')

    # 기본 섹터 추출
    assert abs(sector_min(ranges, angle_min, inc, 90, 10) - 0.3) < 1e-9
    assert abs(sector_min(ranges, angle_min, inc, -90, 10) - 5.0) < 1e-9

    # ±180° 경계를 감아서 찾아야 한다 (후방 섹터)
    assert abs(sector_min(ranges, angle_min, inc, 180, 10) - 0.5) < 1e-9

    # 좁은 섹터는 옆 물체를 못 본다
    assert abs(sector_min(ranges, angle_min, inc, 60, 5) - 5.0) < 1e-9

    # inf/nan은 버리고 주변 유효값을 쓴다
    assert abs(sector_min(ranges, angle_min, inc, 0, 5) - 5.0) < 1e-9

    # range_min 미만은 노이즈로 버린다
    noisy = [0.01] * n
    assert sector_min(noisy, angle_min, inc, 0, 10, range_min=0.12) is None

    # 전부 무효면 None
    assert sector_min([float('inf')] * n, angle_min, inc, 0, 10) is None

    # is_clear
    assert is_clear(ranges, angle_min, inc, -90, 10, clear_dist=1.0) is True
    assert is_clear(ranges, angle_min, inc, 90, 10, clear_dist=1.0) is False
    assert is_clear([], angle_min, inc, 0, 10, clear_dist=1.0, unknown_is_clear=False) is False

    # 우측 차로 영역: box = x -5.72~0.22, y -0.79~-0.19
    box = right_lane_zone()
    assert all(abs(a - b) < 1e-9 for a, b in zip(box, (-5.72, 0.22, -0.79, -0.19))), box

    def scan_with(points):  # [(각도deg, 거리)] 외에는 전부 무반사
        rs = [float('inf')] * n
        for deg, r in points:
            rs[idx(deg)] = r
        return rs

    def polar(x, y):
        return math.degrees(math.atan2(y, x)), math.hypot(x, y)

    # 우측 옆 0.4m → 영역 안
    hit = zone_nearest(scan_with([polar(0.0, -0.4)]), angle_min, inc, box)
    assert hit is not None and abs(hit[2] + 0.4) < 0.02
    # 우후방 4m 뒤, 0.4m 옆 → 영역 안 (부채꼴 ±30° 두 개였으면 사이 틈에 빠지던 위치)
    assert zone_nearest(scan_with([polar(-4.0, -0.4)]), angle_min, inc, box) is not None
    # 자기 차로 바로 뒤 2m → 영역 밖 (R79 영역은 자기 차로 뒤를 포함하지 않음)
    assert zone_nearest(scan_with([polar(-2.0, 0.0)]), angle_min, inc, box) is None
    # 우측 1.25m (지난 실측: 인도 쪽 구조물) → 차로 폭 밖이라 영역 밖
    assert zone_nearest(scan_with([polar(0.0, -1.25)]), angle_min, inc, box) is None
    # 6m 뒤 → 영역 밖
    assert zone_nearest(scan_with([polar(-6.0, -0.4)]), angle_min, inc, box) is None
    # 좌측 → 영역 밖
    assert zone_nearest(scan_with([polar(-1.0, 0.4)]), angle_min, inc, box) is None

    print("scan_sectors self-check OK")


if __name__ == '__main__':
    _self_check()
