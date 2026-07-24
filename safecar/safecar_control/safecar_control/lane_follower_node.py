import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist


class LaneFollowerNode(Node):
    """'/perception/lane_offset'을 받아 차선 중앙을 따라가는 주행 명령을 만든다. (제어부)

    DecisionMaker로부터 '/control/driving_state'를 구독하여,
    NORMAL일 경우 중앙 추종, MRM_PULL_OVER일 경우 갓길(우측) 편향 및 감속 정차를 수행한다.
    """

    def __init__(self):
        super().__init__('lane_follower_node')

        # 기존 자율주행 파라미터
        self.declare_parameter('cruise_speed', 0.12)   # 전진 속도 m/s
        self.declare_parameter('steer_gain', 0.8)      # 조향 P게인 rad/s (과보정/좌우진동 방지 위해 하향)
        self.declare_parameter('steer_d_gain', 0.3)    # 조향 D게인 (핑퐁 방지)
        self.declare_parameter('steer_deadband', 0.05) # 이 이내 오차는 무시(중앙 근처 미세 떨림 방지)
        self.declare_parameter('max_steer', 1.0)       # 조향 각속도 상한 rad/s (과조향 클램프)
        self.declare_parameter('offset_timeout', 0.5)  # 차선 유실 판정 시간(초)
        self.declare_parameter('offset_smoothing', 0.8)# 오프셋 저역통과 필터 계수(높을수록 부드럽게)
        
        # [추가] MRM 전용 파라미터
        # target_offset이 음수이면 차로 중심이 화면 좌측에 오도록 유도 -> 즉, 차는 우측 갓길로 이동함
        self.declare_parameter('mrm_target_offset', -0.6) 
        self.declare_parameter('mrm_speed_ratio', 0.6)     # MRM 진입 시 감속 비율 (예: 60% 속도)
        self.declare_parameter('mrm_stop_duration', 3.0)   # 우측에 붙은 상태를 유지 후 완전 정지하기까지의 시간(초)

        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.steer_gain = self.get_parameter('steer_gain').value
        self.steer_d_gain = self.get_parameter('steer_d_gain').value
        self.steer_deadband = self.get_parameter('steer_deadband').value
        self.max_steer = self.get_parameter('max_steer').value
        self.offset_timeout = self.get_parameter('offset_timeout').value
        self.offset_smoothing = self.get_parameter('offset_smoothing').value
        
        self.mrm_target_offset = self.get_parameter('mrm_target_offset').value
        self.mrm_speed_ratio = self.get_parameter('mrm_speed_ratio').value
        self.mrm_stop_duration = self.get_parameter('mrm_stop_duration').value

        # 상태 변수
        self.last_offset = 0.0
        self.prev_error = 0.0
        self.last_offset_time = None
        self.following = False
        
        self.driving_state = 'NORMAL'
        self.mrm_reached_edge_time = None  # 갓길 도달 시점 기록용

        # Sub / Pub / Timer 설정
        self.create_subscription(String, '/control/driving_state', self._on_state, 10)
        self.create_subscription(Float32, '/perception/lane_offset', self._on_offset, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.create_timer(0.05, self._on_timer)  # 20Hz

    def _on_state(self, msg):
        """DecisionMaker의 상태 변경을 감지한다."""
        if self.driving_state != msg.data:
            self.get_logger().info(f'주행 상태 변경: {self.driving_state} -> {msg.data}')
            self.driving_state = msg.data
            
            # 상태가 전환되면 정차 타이머 초기화
            self.mrm_reached_edge_time = None

    def _on_offset(self, msg):
        now = self.get_clock().now()
        stale = (
            self.last_offset_time is None
            or (now - self.last_offset_time).nanoseconds * 1e-9 > self.offset_timeout
        )
        
        if stale:
            self.last_offset = msg.data
            self.prev_error = 0.0  # 목표 오프셋이 개입되므로 재시작 시 오차 초기화
        else:
            a = self.offset_smoothing
            self.last_offset = a * self.last_offset + (1.0 - a) * msg.data
            
        self.last_offset_time = now

    def _on_timer(self):
        fresh = (
            self.last_offset_time is not None
            and (self.get_clock().now() - self.last_offset_time).nanoseconds * 1e-9 < self.offset_timeout
        )
        
        if fresh != self.following:
            if fresh:
                self.get_logger().info('차선 추종 시작')
            else:
                self.get_logger().warn(f'차선 {self.offset_timeout:.1f}초 이상 유실 — 정지')
            self.following = fresh

        cmd = Twist()
        
        # 1. 최우선 예외 처리: 긴급 제동
        if self.driving_state == 'EMERGENCY_BRAKE':
            self.cmd_pub.publish(cmd)  # 속도 0 발행
            return

        # 2. 자율 주행 로직 실행
        if fresh:
            target_offset = 0.0
            target_speed = self.cruise_speed
            
            # [MRM_PULL_OVER 로직]
            if self.driving_state == 'MRM_PULL_OVER':
                target_offset = self.mrm_target_offset
                target_speed = self.cruise_speed * self.mrm_speed_ratio
                
                # 갓길에 충분히 진입했는지 확인 (예: offset이 -0.5 이하로 떨어짐)
                if self.last_offset <= target_offset + 0.1:
                    now = self.get_clock().now()
                    if self.mrm_reached_edge_time is None:
                        self.mrm_reached_edge_time = now
                    else:
                        elapsed = (now - self.mrm_reached_edge_time).nanoseconds * 1e-9
                        if elapsed > self.mrm_stop_duration:
                            target_speed = 0.0  # 지정 시간 경과 후 완전 정지
                            self.get_logger().info('MRM 갓길 정차 완료', once=True)
                else:
                    # 차체가 흔들려 다시 갓길 기준을 벗어나면 정차 타이머 리셋
                    self.mrm_reached_edge_time = None 
            
            # 주행 속도가 0.0보다 클 때만 조향 제어 수행
            if target_speed > 0.0:
                cmd.linear.x = target_speed
                
                # 목표 오프셋 대비 현재 오차 계산 (P 제어용)
                # target이 -0.6이고 현재가 0.0이면, 오차는 +0.6 -> 차체를 우측으로 강하게 조향
                current_error = self.last_offset - target_offset

                # 데드밴드: 중앙에 충분히 가까우면 조향하지 않는다(미세한 좌우 떨림 억제)
                if abs(current_error) < self.steer_deadband:
                    current_error = 0.0

                # 오차의 변화량 (D 제어용)
                error_diff = current_error - self.prev_error

                # 최종 조향값 계산: 오차가 양수(+)면 우조향(-) 필요
                steer = -(self.steer_gain * current_error) - (self.steer_d_gain * error_diff)
                # 과조향 클램프
                cmd.angular.z = max(-self.max_steer, min(self.max_steer, steer))

                self.prev_error = current_error
            else:
                # 완전 정지 상태 유지
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                
        self.cmd_pub.publish(cmd)


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