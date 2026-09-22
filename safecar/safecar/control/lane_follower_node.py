import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from safecar.protocol import COMMAND_MRM_PULL_OVER, COMMAND_NORMAL
from safecar.control.mrm_profile import MrmProfile
from safecar.control.scan_sectors import sector_min


class LaneFollowerNode(Node):
    """'/perception/lane_offset'을 받아 차선 중앙을 따라가는 주행 명령을 만든다. (제어부)

    '/cmd_vel'이 아니라 '/cmd_vel_raw'로 publish한다 — 모든 주행 명령은
    decision_maker의 안전 게이트를 거치므로, 장애물(EMERGENCY_BRAKE) 시에는
    이 노드가 계속 발행해도 차는 멈춘다.

    차선을 offset_timeout 이상 못 받으면(차선 유실) 정지 명령을 발행한다.

    갓길 대피(MRM): '/control/driving_state'가 MRM_PULL_OVER가 되면 MrmProfile에 따라
    횡방향으로 치우치며 감속 → 정지한다. 게이트(decision_maker)는 MRM 동안 이 노드의
    명령을 그대로 통과시키므로, 실제 대피 주행을 만드는 건 이 노드다.
    """

    def __init__(self):
        super().__init__('lane_follower_node')

        # 기본값은 2026-09-21 실외 트랙 실측값이다. 실행 경로(launch/스크립트)와 무관하게
        # 같은 값으로 달리도록 튜닝값을 여기에 둔다.
        # 전진 속도 m/s. 0.10 이하는 정지 마찰을 못 이겨 아예 안 움직인다(실측 최소 0.12).
        self.declare_parameter('cruise_speed', 0.15)
        # 조향 P게인 rad/s per offset(-1~+1). 1.2는 발산 진동했다(회전반경 8cm).
        self.declare_parameter('steer_gain', 0.55)
        self.declare_parameter('offset_timeout', 0.5)  # 차선 유실 판정 시간(초)
        # 오프셋 저역통과(EMA) 계수 0~1. 클수록 이전 값 비중이 커져 조향이 부드럽다.
        # 프레임별 검출 노이즈(±0.1~0.2)가 그대로 조향에 실리는 것을 막는다.
        self.declare_parameter('offset_smoothing', 0.7)
        # 헤딩(차선 기울기) 게인. 횡오차만 보는 P 제어는 차가 옆으로 밀린 **뒤에야**
        # 반응하므로 곡선에서 구조적으로 밀린다. 차선이 기울어진 것을 보고 미리 꺾는
        # 이 항이 더해지면 Stanley 제어와 같은 구조가 된다.
        #   angular.z = -(steer_gain*오프셋 + steer_head_gain*헤딩)
        # 0으로 두면 기존의 횡오차 전용 P 제어로 돌아간다.
        # 실측(2026-09-21): 직선에서 헤딩≈0.02, 곡선에서 1.0까지 나온다. 게인 0.5면
        # 곡선에서 0.5 rad/s가 추가되는데, 그날 안정 범위는 최대 0.17이었다.
        # 그래서 0.25에서 시작하고 곡선을 보며 올린다.
        self.declare_parameter('steer_head_gain', 0.25)

        # --- 갓길 대피(MRM) 프로파일 ---
        # mrm_lateral_bias: 우차로정차를 켤지 끄는 스위치. 0.0이면 무조건 차로 안에서
        # 그대로 정지(자차로정차) — 이게 UN R157의 기본 MRM이다. 0이 아니면(값 자체는
        # 더 이상 안 씀) _decide_mrm_mode가 라이다로 후방·우측 여유를 보고 우차로정차를
        # 시도한다. 실제 조향은 이 숫자가 아니라 갓길(노란 테이프) 실시간 인식이 맡는다
        # (vision_detector.set_shoulder_mode, _mrm_cmd 참고). 본 차량은 후방 감시 센서가
        # 없어 차선 변경형 대피를 할 자격이 없으므로, 0이 아닌 값은 '후방 교통 없는
        # 폐쇄 트랙'이라는 ODD 안에서만 쓴다. ODD를 벗어나면 0.0으로 되돌릴 것.
        self.declare_parameter('mrm_lateral_bias', 0.5)
        self.declare_parameter('mrm_transition_time', 3.0)  # 갓길로 붙는 시간(초)
        # 대피 중 속도 비율. **하한이 하드웨어로 정해져 있다** — 실측(2026-09-21) 결과
        # 0.12 m/s 아래로는 정지 마찰을 못 이겨 차가 아예 안 움직인다.
        # cruise 0.15에 ratio 0.6이면 이동 구간에서 0.09까지 떨어져 갓길에 닿기 전에
        # 멈춰버린다. 0.85면 이동 내내 0.1275 이상을 유지한다.
        self.declare_parameter('mrm_speed_ratio', 0.85)
        self.declare_parameter('mrm_stop_duration', 2.0)    # 붙은 뒤 정지까지(초)

        # --- 라이다(/scan)로 갓길 대피 가능 여부 판단 ---
        # 섹터 중심각은 라이다 장착 방향에 따라 달라진다. 차 우측/뒤에만 물체를 두고
        # 어느 중심각에서 거리가 줄어드는지 확인한 뒤 맞출 것(README의 검증 절차 참고).
        self.declare_parameter('mrm_rear_deg', 180.0)       # 후방 섹터 중심각
        self.declare_parameter('mrm_side_deg', -90.0)       # 갓길(우측) 섹터 중심각
        self.declare_parameter('mrm_sector_half_deg', 30.0)  # 섹터 반각
        self.declare_parameter('mrm_rear_clear_m', 0.6)     # 후방이 이보다 비어야 대피 허용
        self.declare_parameter('mrm_side_clear_m', 0.35)    # 갓길쪽이 이보다 비어야 대피 허용
        self.declare_parameter('mrm_scan_timeout', 1.0)     # /scan 신선도(초)
        # /scan을 못 받으면 갓길 대피를 포기하고 차로 내 정지로 간다(R157 기본 동작).
        # 라이다 없이 대피 동작만 튜닝할 때는 false로 끈다.
        self.declare_parameter('mrm_require_scan', True)

        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.steer_gain = self.get_parameter('steer_gain').value
        self.offset_timeout = self.get_parameter('offset_timeout').value
        self.offset_smoothing = self.get_parameter('offset_smoothing').value
        self.steer_head_gain = self.get_parameter('steer_head_gain').value
        self.mrm_lateral_bias = self.get_parameter('mrm_lateral_bias').value
        self.mrm = MrmProfile(
            lateral_bias=self.mrm_lateral_bias,
            transition_time=self.get_parameter('mrm_transition_time').value,
            speed_ratio=self.get_parameter('mrm_speed_ratio').value,
            stop_duration=self.get_parameter('mrm_stop_duration').value,
        )
        self.mrm_rear_deg = self.get_parameter('mrm_rear_deg').value
        self.mrm_side_deg = self.get_parameter('mrm_side_deg').value
        self.mrm_sector_half_deg = self.get_parameter('mrm_sector_half_deg').value
        self.mrm_rear_clear_m = self.get_parameter('mrm_rear_clear_m').value
        self.mrm_side_clear_m = self.get_parameter('mrm_side_clear_m').value
        self.mrm_scan_timeout = self.get_parameter('mrm_scan_timeout').value
        self.mrm_require_scan = self.get_parameter('mrm_require_scan').value

        self.last_offset = 0.0
        self.last_offset_time = None
        self.last_heading = 0.0
        self.following = False
        self.mrm_start_time = None  # None이면 대피 중이 아님
        self.mrm_mode = None        # 이번 대피에서 선택된 MRM 모드명
        self.last_scan = None
        self.last_scan_time = None

        self.create_subscription(Float32, '/perception/lane_offset', self._on_offset, 10)
        self.create_subscription(Float32, '/perception/lane_heading', self._on_heading, 10)
        self.create_subscription(String, '/control/driving_state', self._on_state, 10)
        # ydlidar는 SensorDataQoS(BEST_EFFORT)로 발행한다. 예전엔 기본 QoS(RELIABLE)로
        # 구독해서 호환이 안 됐고, /scan을 한 번도 못 받아 MRM이 항상 '/scan 없음 →
        # 자차로정차'로 떨어지고 있었다.
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        # vision_detector_node가 이걸 보고 우차로정차일 때만 갓길(노란선) 추적으로 바꾼다.
        # driving_state만으로는 자차로정차/우차로정차를 구분 못 해서 따로 publish한다.
        self.mrm_mode_pub = self.create_publisher(String, '/control/mrm_mode', 10)
        self.create_timer(0.05, self._on_timer)  # 20Hz

    def _on_offset(self, msg):
        now = self.get_clock().now()
        stale = (
            self.last_offset_time is None
            or (now - self.last_offset_time).nanoseconds * 1e-9 > self.offset_timeout
        )
        if stale:
            self.last_offset = msg.data  # 차선 재획득: 옛 값과 섞지 않고 새로 시작
        else:
            a = self.offset_smoothing
            self.last_offset = a * self.last_offset + (1.0 - a) * msg.data
        self.last_offset_time = now

    def _on_scan(self, msg):
        self.last_scan = msg
        self.last_scan_time = self.get_clock().now()

    def _decide_mrm_mode(self, lane_fresh):
        """대피 시작 시점에 수행할 MRM 모드를 정한다. → (모드명, 횡 바이어스)

        선행특허 KR 10-2024-0073259(ETRI, '자율주행을 위한 MRM 장치와 방법 및 MRM 모드
        결정 방법') 청구항 13~16의 결정 플로우를 이 차량의 센서 구성에 맞춰 구현한 것이다.

            차로 유지 가능?  ─아니오─▶ 직진정차   (조향 끊고 감속만)
                 │예
            차로 변경 가능?  ─아니오─▶ 자차로정차 (차로 안에서 정지)
                 │예
            갓길 존재?       ─아니오─▶ 우차로정차 (우측으로 붙어 정지)
                 │예
                 └────────────────▶ 갓길주차

        특허가 정의한 6모드 중 이 차량이 수행 가능한 범위:
        - 비상정차   : 구현됨 — decision_maker의 EMERGENCY_BRAKE(전방 장애물 시 즉시 정지)
        - 직진정차   : 구현됨 — 대피 중 차선 유실 시 `_mrm_cmd`가 자동 수행
        - 자차로정차 : 구현됨 — bias=0
        - 우차로정차 : 구현됨 — bias>0 (현재 시연이 도달하는 모드)
        - 갓길주차   : 미구현 — 갓길 존재 판정 수단이 없다(트랙에 갓길 표시 필요)
        - 안전지대주차: 범위 밖 — HD맵/V2X 필요

        특허 [0025]의 원칙(감지 범위·거리가 제한적이면 더 낮은 단계의 MRM을 선택)을 따라,
        후방/측방을 확인할 수 없으면 한 단계 낮은 모드로 내려간다.

        ponytail: 대피 시작 시점에 한 번만 판단한다(이동 중 뒤차가 새로 접근하는 건 못 본다).
        연속 감시로 올리려면 `_mrm_cmd`에서 매 주기 재평가하고 중단 조건을 넣어야 한다.
        """
        # ① 차로 유지 가능한가 (특허 203) — 차선이 안 보이면 횡방향 제어 자체가 불가능
        if not lane_fresh:
            self.get_logger().warn('MRM 모드: 직진정차 (차선 유지 불가)')
            return '직진정차', 0.0

        if self.mrm_lateral_bias == 0.0:
            self.get_logger().warn('MRM 모드: 자차로정차 (설정값)')
            return '자차로정차', 0.0

        # ② 차로 변경(횡이동) 가능한가 (특허 211) — 후방/우측방 여유 확인
        fresh = (
            self.last_scan is not None
            and self.last_scan_time is not None
            and (self.get_clock().now() - self.last_scan_time).nanoseconds * 1e-9
            < self.mrm_scan_timeout
        )
        if not fresh:
            if self.mrm_require_scan:
                self.get_logger().warn('MRM 모드: 자차로정차 (/scan 없음 — 횡이동 안전 미확인)')
                return '자차로정차', 0.0
            self.get_logger().warn('MRM 모드: 우차로정차 (/scan 확인 생략)')
            return '우차로정차', self.mrm_lateral_bias

        s = self.last_scan
        rear = sector_min(s.ranges, s.angle_min, s.angle_increment,
                          self.mrm_rear_deg, self.mrm_sector_half_deg,
                          s.range_min, s.range_max)
        side = sector_min(s.ranges, s.angle_min, s.angle_increment,
                          self.mrm_side_deg, self.mrm_sector_half_deg,
                          s.range_min, s.range_max)

        def fmt(d):
            return '비어있음' if d is None else f'{d:.2f}m'

        # None = 그 방향에 반사가 없음 = 비어 있음.
        rear_ok = rear is None or rear > self.mrm_rear_clear_m
        side_ok = side is None or side > self.mrm_side_clear_m
        if not (rear_ok and side_ok):
            self.get_logger().warn(
                f'MRM 모드: 자차로정차 (횡이동 불가 — 후방 {fmt(rear)}, 우측 {fmt(side)})')
            return '자차로정차', 0.0

        # ③ 갓길 존재 판정 (특허 227) — 현재 판정 수단이 없어 항상 '없음'으로 본다.
        #    트랙에 갓길 표시를 붙이고 인지부가 알려주면 여기서 '갓길주차'로 올라간다.
        self.get_logger().warn(
            f'MRM 모드: 우차로정차 (후방 {fmt(rear)}, 우측 {fmt(side)})')
        return '우차로정차', self.mrm_lateral_bias

    def _on_heading(self, msg):
        # 오프셋과 같은 계수로 평활한다. 둘 다 같은 검출에서 나오므로 잡음 특성이 비슷하다.
        a = self.offset_smoothing
        self.last_heading = a * self.last_heading + (1.0 - a) * msg.data

    def _on_state(self, msg):
        mrm = (msg.data == COMMAND_MRM_PULL_OVER)
        if mrm and self.mrm_start_time is None:
            lane_fresh = (
                self.last_offset_time is not None
                and (self.get_clock().now() - self.last_offset_time).nanoseconds * 1e-9
                < self.offset_timeout
            )
            self.mrm_mode, self.mrm.lateral_bias = self._decide_mrm_mode(lane_fresh)
            self.mrm_start_time = self.get_clock().now()
            self.mrm_mode_pub.publish(String(data=self.mrm_mode))
            self.get_logger().warn(f'운전자 이상 — MRM 시작 ({self.mrm_mode})')
        # NORMAL일 때만 해제한다. EMERGENCY_BRAKE(장애물·인지 끊김)는 MRM 도중에도 잠깐 끼어들
        # 수 있는데, 그걸 해제로 보면 장애물이 사라진 뒤 프로파일이 0초부터 다시 시작돼
        # 정차했던 차가 재출발한다(R157 위반). 비상정지 동안에도 프로파일 시간은 계속 흐르므로
        # 이미 정차를 마친 뒤라면 장애물이 사라져도 속도 0이 유지된다.
        elif msg.data == COMMAND_NORMAL and self.mrm_start_time is not None:
            self.mrm_start_time = None
            self.mrm_mode = None
            self.mrm_mode_pub.publish(String(data=''))  # vision_detector: 흰 차선으로 복귀
            self.get_logger().info('MRM 해제 — 정상 주행 복귀')

    def _on_timer(self):
        fresh = (
            self.last_offset_time is not None
            and (self.get_clock().now() - self.last_offset_time).nanoseconds * 1e-9
            < self.offset_timeout
        )

        if self.mrm_start_time is not None:
            self.cmd_pub.publish(self._mrm_cmd(fresh))
            return

        if fresh != self.following:
            if fresh:
                self.get_logger().info('차선 추종 시작')
            else:
                self.get_logger().warn(f'차선 {self.offset_timeout:.1f}초 이상 유실 — 정지')
            self.following = fresh

        cmd = Twist()
        if fresh:
            cmd.linear.x = self.cruise_speed
            # REP 103: angular.z +는 좌회전. offset·heading 모두 +가 '우조향 필요'이므로 부호 반전.
            cmd.angular.z = -(self.steer_gain * self.last_offset
                              + self.steer_head_gain * self.last_heading)
        self.cmd_pub.publish(cmd)

    def _mrm_cmd(self, lane_fresh):
        """갓길 대피 주행 명령. 차선을 잃어도 중단하지 않고 프로파일을 끝까지 수행한다.

        우차로정차 중에는 vision_detector가 '/control/mrm_mode'를 보고 흰 차선 대신
        갓길(노란 테이프)을 추적하도록 이미 전환해뒀다 — 그래서 여기서는 오프셋에 가짜
        bias를 더해 속이지 않는다. last_offset이 이제 '갓길선까지의 오프셋'이므로,
        평소 차선 추종과 똑같이 그 값을 0으로 만들면(= 갓길선에 정렬하면) 저절로
        갓길로 붙는다. MrmProfile은 속도 감속 곡선(1.0→speed_ratio→0)에만 쓴다.
        """
        elapsed = (self.get_clock().now() - self.mrm_start_time).nanoseconds * 1e-9
        speed_scale, _ = self.mrm.compute(elapsed)

        cmd = Twist()
        cmd.linear.x = self.cruise_speed * speed_scale
        if lane_fresh:
            cmd.angular.z = -(self.steer_gain * self.last_offset
                              + self.steer_head_gain * self.last_heading)
        else:
            # 추종 대상(흰 차선 또는 갓길선)을 잃은 상태. 마지막 오프셋을 계속 믿으면
            # 그대로 돌아버리므로 조향을 끊고 직진으로 감속만 이어간다.
            cmd.angular.z = 0.0
        return cmd


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
