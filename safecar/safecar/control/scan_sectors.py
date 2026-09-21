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

    print("scan_sectors self-check OK")


if __name__ == '__main__':
    _self_check()
