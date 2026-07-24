import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String
from geometry_msgs.msg import Twist


class LaneFollowerNode(Node):
    """'/perception/lane_path'(정규화 중심선)을 받아 Pure Pursuit로 차선을 추종한다. (제어부)

    - NORMAL: 중심선을 Pure Pursuit로 부드럽게 추종, 곡률에 따라 자동 감속.
    - MRM_PULL_OVER: 목표 중심선을 우측으로 램프(호를 그리며 이동)한 뒤 램프 감속 정차.
    - EMERGENCY_BRAKE: 즉시 정지.
    출력은 '/cmd_vel_raw' (안전 게이트 decision_maker가 '/cmd_vel'로 통과시킴).

    좌표 규약(인지부와 공유): path 점 (lateral, forward) 정규화. lateral +는 오른쪽,
    forward +는 전방, 스케일 1.0 = warp 반폭. offset도 동일 스케일.
    """

    def __init__(self):
        super().__init__('lane_follower_node')

        # 주행 파라미터
        self.declare_parameter('cruise_speed', 0.12)    # 전진 속도 m/s
        self.declare_parameter('v_min', 0.06)           # 곡선 감속 하한 m/s
        self.declare_parameter('lookahead_dist', 0.8)   # Pure Pursuit 전방 거리(정규화, 최대 ~0.97)
        self.declare_parameter('steer_gain', 0.8)       # 전체 조향 세기
        self.declare_parameter('steer_deadband', 0.05)  # 이 이내 횡오차는 무시
        self.declare_parameter('max_steer', 1.0)        # 조향 각속도 상한 rad/s
        self.declare_parameter('k_curv', 1.0)           # 곡률 기반 감속 세기
        self.declare_parameter('max_ang_accel', 4.0)    # 조향 급변 제한 rad/s^2
        self.declare_parameter('max_lin_accel', 0.5)    # 가감속 제한 m/s^2
        self.declare_parameter('path_timeout', 0.5)     # 차선 유실 판정 시간(초)

        # MRM(갓길 대피) 파라미터
        self.declare_parameter('mrm_target_offset', -0.6)  # 음수 = 우측 갓길로 이동(정규화)
        self.declare_parameter('mrm_speed_ratio', 0.6)     # 대피 중 속도 비율
        self.declare_parameter('mrm_transition_time', 2.0) # 갓길로 붙는 시간(호의 완만함)
        self.declare_parameter('mrm_stop_duration', 3.0)   # 갓길 도달 후 정차까지 시간

        g = self.get_parameter
        self.cruise_speed = g('cruise_speed').value
        self.v_min = g('v_min').value
        self.lookahead_dist = g('lookahead_dist').value
        self.steer_gain = g('steer_gain').value
        self.steer_deadband = g('steer_deadband').value
        self.max_steer = g('max_steer').value
        self.k_curv = g('k_curv').value
        self.max_ang_accel = g('max_ang_accel').value
        self.max_lin_accel = g('max_lin_accel').value
        self.path_timeout = g('path_timeout').value
        self.mrm_target_offset = g('mrm_target_offset').value
        self.mrm_speed_ratio = g('mrm_speed_ratio').value
        self.mrm_transition_time = g('mrm_transition_time').value
        self.mrm_stop_duration = g('mrm_stop_duration').value

        # 상태
        self.last_path = None
        self.last_path_time = None
        self.last_offset = 0.0
        self.last_offset_time = None
        self.following = False
        self.driving_state = 'NORMAL'
        self.mrm_t0 = None            # 갓길 이동 시작 시각
        self.mrm_stop_t0 = None       # 갓길 도달(정차 램프) 시작 시각
        self.prev_ang = 0.0
        self.prev_lin = 0.0

        self._dt = 0.05

        self.create_subscription(String, '/control/driving_state', self._on_state, 10)
        self.create_subscription(Float32MultiArray, '/perception/lane_path', self._on_path, 10)
        self.create_subscription(Float32, '/perception/lane_offset', self._on_offset, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.create_timer(self._dt, self._on_timer)  # 20Hz

    # ── 콜백 ────────────────────────────────────────────────────────
    def _on_state(self, msg):
        if self.driving_state != msg.data:
            self.get_logger().info(f'주행 상태 변경: {self.driving_state} -> {msg.data}')
            self.driving_state = msg.data
            self.mrm_t0 = None
            self.mrm_stop_t0 = None

    def _on_path(self, msg):
        data = msg.data
        if len(data) >= 2:
            self.last_path = [(data[i], data[i + 1]) for i in range(0, len(data) - 1, 2)]
            self.last_path_time = self.get_clock().now()

    def _on_offset(self, msg):
        self.last_offset = msg.data
        self.last_offset_time = self.get_clock().now()

    # ── 유틸 ────────────────────────────────────────────────────────
    def _fresh(self, t):
        return (t is not None
                and (self.get_clock().now() - t).nanoseconds * 1e-9 < self.path_timeout)

    @staticmethod
    def _rate_limit(prev, target, max_delta):
        return prev + max(-max_delta, min(max_delta, target - prev))

    def _pick_target(self, path, ld):
        """forward 오름차순 path에서 lookahead 거리 ld에 해당하는 (lateral, forward) 보간."""
        for i, (lat, fwd) in enumerate(path):
            if fwd >= ld:
                if i == 0:
                    return lat, fwd
                lat0, fwd0 = path[i - 1]
                t = (ld - fwd0) / (fwd - fwd0) if fwd != fwd0 else 0.0
                return lat0 + t * (lat - lat0), ld
        return path[-1]  # path가 짧으면 가장 먼 점

    def _publish(self, lin, ang):
        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self.prev_lin, self.prev_ang = lin, ang
        self.cmd_pub.publish(cmd)

    # ── 20Hz 제어 루프 ─────────────────────────────────────────────
    def _on_timer(self):
        # 1) 긴급 제동: 즉시 정지
        if self.driving_state == 'EMERGENCY_BRAKE':
            self._publish(0.0, 0.0)
            return

        # 2) 목표 결정: path 우선, 없으면 offset 스칼라로 대체
        path = None
        if self._fresh(self.last_path_time) and self.last_path:
            path = self.last_path
        elif self._fresh(self.last_offset_time):
            path = [(self.last_offset, self.lookahead_dist)]

        fresh = path is not None
        if fresh != self.following:
            self.get_logger().info('차선 추종 시작' if fresh
                                   else f'차선 {self.path_timeout:.1f}초 이상 유실 — 정지')
            self.following = fresh

        # 3) 차선 유실: 부드럽게 감속 정지
        if not fresh:
            lin = self._rate_limit(self.prev_lin, 0.0, self.max_lin_accel * self._dt)
            ang = self._rate_limit(self.prev_ang, 0.0, self.max_ang_accel * self._dt)
            self._publish(lin, ang)
            return

        target_lat, target_fwd = self._pick_target(path, self.lookahead_dist)

        # 4) MRM: 목표를 우측으로 램프 이동 + 램프 감속
        base_speed = self.cruise_speed
        if self.driving_state == 'MRM_PULL_OVER':
            now = self.get_clock().now()
            if self.mrm_t0 is None:
                self.mrm_t0 = now
            ramp = min(1.0, (now - self.mrm_t0).nanoseconds * 1e-9 / max(1e-3, self.mrm_transition_time))
            target_lat += ramp * (-self.mrm_target_offset)  # 음수 offset -> 우측(+) 바이어스

            if ramp >= 1.0:  # 갓길 도달 -> 정차 램프
                if self.mrm_stop_t0 is None:
                    self.mrm_stop_t0 = now
                st = (now - self.mrm_stop_t0).nanoseconds * 1e-9
                stop_ratio = max(0.0, 1.0 - st / max(1e-3, self.mrm_stop_duration))
                if stop_ratio <= 0.0:
                    self.get_logger().info('MRM 갓길 정차 완료', once=True)
            else:
                stop_ratio = 1.0
            base_speed = self.cruise_speed * self.mrm_speed_ratio * stop_ratio

        # 5) Pure Pursuit 곡률 + 조향
        lat_eff = 0.0 if abs(target_lat) < self.steer_deadband else target_lat
        ld = math.hypot(lat_eff, target_fwd)
        ld = max(0.1, ld)
        gamma = 2.0 * lat_eff / (ld * ld)          # 중심선 곡률(정규화)
        omega = -self.steer_gain * gamma           # +lateral(오른쪽) -> 우조향(-)
        omega = max(-self.max_steer, min(self.max_steer, omega))

        # 6) 곡률 기반 감속
        v = base_speed / (1.0 + self.k_curv * abs(gamma))
        v = max(min(self.v_min, base_speed), min(v, base_speed))  # base가 0에 수렴하면 0까지 허용

        # 7) rate limit로 부드럽게
        lin = self._rate_limit(self.prev_lin, v, self.max_lin_accel * self._dt)
        ang = self._rate_limit(self.prev_ang, omega, self.max_ang_accel * self._dt)
        self._publish(lin, ang)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
