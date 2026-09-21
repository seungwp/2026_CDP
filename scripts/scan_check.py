#!/usr/bin/env python3
"""라이다 진단용 — /scan을 30° 구간으로 잘라 최소거리를 1초마다 출력한다.

두 가지를 한 번에 확인한다:
  1) 가림(occlusion) — 아무것도 없는 곳인데 특정 구간만 항상 짧으면 자기 차체/구조물에 가린 것.
  2) 각도 대응     — 차 옆이나 뒤에만 물체를 두고, 어느 구간의 거리가 줄어드는지 본다.
                     그 각도가 `lane_follower_node`의 mrm_side_deg / mrm_rear_deg 값이다.
                     (YDLIDAR의 reversion 설정 때문에 ROS 표준과 다를 수 있어 실측이 필요하다.)

colcon build 없이 그냥 실행한다 (패키지가 아니라 단독 스크립트):
    source install/setup.bash && export ROS_DOMAIN_ID=52
    python3 scripts/scan_check.py

로직만 확인하려면:  python3 scripts/scan_check.py --selftest
"""

import math
import sys

BUCKET_DEG = 30.0


def bucket_mins(ranges, angle_min, angle_increment, range_min, range_max,
                bucket_deg=BUCKET_DEG):
    """[(구간중심각, 반각, 최소거리 또는 None), ...] — 전체 360°를 bucket_deg로 자른다.

    버킷은 bucket_deg의 배수를 **중심**으로 잡는다 — 그래야 0/±90/180(전·좌·우·후)이
    경계에 걸려 두 칸으로 쪼개지지 않고, 읽은 중심각을 그대로 파라미터에 넣을 수 있다.
    """
    n = int(round(360.0 / bucket_deg))
    half = bucket_deg / 2.0
    mins = [None] * n

    for i, r in enumerate(ranges):
        if r is None or math.isnan(r) or math.isinf(r):
            continue
        if not (range_min <= r <= range_max):
            continue
        deg = math.degrees(angle_min + i * angle_increment)
        b = int(round(deg / bucket_deg)) % n
        if mins[b] is None or r < mins[b]:
            mins[b] = r

    # 인덱스 → 중심각을 [-180, 180) 으로
    return [((k * bucket_deg + 180.0) % 360.0 - 180.0, half, mins[k])
            for k in range(n)]


def _label(center):
    """방위 이름 (ROS 표준 기준: 전방 0°, 좌측 +90°, 우측 -90°, 후방 ±180°)."""
    return {0.0: '전방', 90.0: '좌측', -90.0: '우측', 180.0: '후방',
            -180.0: '후방'}.get(round(center, 3), '')


def render(rows):
    out = []
    valid = [d for _, _, d in rows if d is not None]
    closest = min(valid) if valid else None
    for center, half, d in sorted(rows):
        name = _label(center)
        if d is None:
            body = '     --   (반사 없음 = 비어있음)'
        else:
            bar = '#' * max(1, int(round(20 * (1.0 - min(d, 2.0) / 2.0))))
            mark = '  <== 최근접' if d == closest else ''
            body = f'{d:7.2f}m  {bar}{mark}'
        out.append(f'중심 {center:+7.1f}° (±{half:.0f}°) {name:>4} : {body}')
    return '\n'.join(out)


def _self_check():
    n = 360
    angle_min = -math.pi
    inc = 2.0 * math.pi / n
    ranges = [5.0] * n
    ranges[int(round((math.radians(-90) - angle_min) / inc))] = 0.3  # 우측에 물체

    rows = bucket_mins(ranges, angle_min, inc, 0.12, 10.0)
    assert len(rows) == 12
    hit = [r for r in rows if r[2] is not None and r[2] < 1.0]
    assert len(hit) == 1, hit
    center, half, d = hit[0]
    # 우측(-90°)에 둔 물체는 -90°를 '중심'으로 하는 칸에 잡혀야 한다(경계에 쪼개지지 않음)
    assert abs(center + 90.0) < 1e-9 and abs(d - 0.3) < 1e-9, (center, d)
    assert _label(center) == '우측'
    assert {_label(c) for c, _, _ in rows} >= {'전방', '좌측', '우측', '후방'}

    # 전부 무반사면 전 구간 None
    assert all(r[2] is None for r in bucket_mins([float('inf')] * n, angle_min, inc, 0.12, 10.0))
    print('scan_check self-check OK')


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    class ScanCheck(Node):
        def __init__(self):
            super().__init__('scan_check')
            self.msg = None
            # ydlidar는 SensorDataQoS(BEST_EFFORT)로 발행한다. 기본 QoS(RELIABLE)로
            # 구독하면 'incompatible QoS'가 뜨고 메시지가 **하나도** 안 들어온다.
            self.create_subscription(LaserScan, '/scan', self._on_scan,
                                     qos_profile_sensor_data)
            self.create_timer(1.0, self._report)
            self.get_logger().info('/scan 대기 중...')

        def _on_scan(self, msg):
            self.msg = msg

        def _report(self):
            m = self.msg
            if m is None:
                self.get_logger().warn('/scan 수신 없음 — 라이다 노드가 떠 있는지 확인')
                return
            rows = bucket_mins(m.ranges, m.angle_min, m.angle_increment,
                               m.range_min, m.range_max)
            print(f'\n=== /scan  점수 {len(m.ranges)}  '
                  f'각도 {math.degrees(m.angle_min):+.1f}~{math.degrees(m.angle_max):+.1f}°  '
                  f'거리 {m.range_min:.2f}~{m.range_max:.1f}m ===')
            print(render(rows))

    rclpy.init()
    node = ScanCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    if '--selftest' in sys.argv:
        _self_check()
    else:
        main()
