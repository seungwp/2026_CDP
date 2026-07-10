# lane_follower_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class LaneFollowerNode(Node):
    """'/perception/lane_offset'을 받아 차선 중앙을 따라가는 주행 명령을 만든다. (제어부)

    '/cmd_vel'이 아니라 '/cmd_vel_raw'로 publish한다 — 모든 주행 명령은
    decision_maker의 안전 게이트를 거치므로, 장애물(EMERGENCY_BRAKE)이나
    운전자 이상(MRM_PULL_OVER) 시에는 이 노드가 계속 발행해도 차는 멈춘다.

    차선을 offset_timeout 이상 못 받으면(차선 유실) 정지 명령을 발행한다.
    """

    def __init__(self):
        super().__init__('lane_follower_node')

        self.declare_parameter('cruise_speed', 0.12)   # 전진 속도 m/s (트랙에서 튜닝)
        self.declare_parameter('steer_gain', 1.2)      # 조향 P게인 rad/s per offset(-1~+1)
        self.declare_parameter('steer_d_gain', 0.3)    # [추가] 조향 D게인 (핑퐁 방지 댐퍼)
        self.declare_parameter('offset_timeout', 0.5)  # 차선 유실 판정 시간(초)
        
        # 오프셋 저역통과(EMA) 계수 0~1. 클수록 이전 값 비중이 커져 조향이 부드럽다.
        # 프레임별 검출 노이즈(±0.1~0.2)가 그대로 조향에 실리는 것을 막는다.
        self.declare_parameter('offset_smoothing', 0.7)
        
        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.steer_gain = self.get_parameter('steer_gain').value
        self.steer_d_gain = self.get_parameter('steer_d_gain').value
        self.offset_timeout = self.get_parameter('offset_timeout').value
        self.offset_smoothing = self.get_parameter('offset_smoothing').value

        self.last_offset = 0.0
        self.prev_error = 0.0 # [추가] 이전 프레임 오차 기억용 변수
        self.last_offset_time = None
        self.following = False

        self.create_subscription(Float32, '/perception/lane_offset', self._on_offset, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.create_timer(0.05, self._on_timer)  # 20Hz

    def _on_offset(self, msg):
        now = self.get_clock().now()
        stale = (
            self.last_offset_time is None
            or (now - self.last_offset_time).nanoseconds * 1e-9 > self.offset_timeout
        )
        if stale:
            self.last_offset = msg.data  # 차선 재획득: 옛 값과 섞지 않고 새로 시작
            self.prev_error = msg.data   # 새로 시작할 때 D게인이 튀지 않도록 초기화
        else:
            a = self.offset_smoothing
            self.last_offset = a * self.last_offset + (1.0 - a) * msg.data
        self.last_offset_time = now

    def _on_timer(self):
        fresh = (
            self.last_offset_time is not None
            and (self.get_clock().now() - self.last_offset_time).nanoseconds * 1e-9
            < self.offset_timeout
        )
        if fresh != self.following:
            if fresh:
                self.get_logger().info('차선 추종 시작')
            else:
                self.get_logger().warn(f'차선 {self.offset_timeout:.1f}초 이상 유실 — 정지')
            self.following = fresh

        cmd = Twist()
        if fresh:
            cmd.linear.x = self.cruise_speed
            
            # 1. 현재 오차 (P 제어용)
            current_error = self.last_offset
            
            # 2. 오차의 변화량 (D 제어용) -> 현재 오차에서 이전 오차를 뺌
            error_diff = current_error - self.prev_error
            
            # 3. P와 D를 합쳐서 최종 조향값 계산!
            # REP 103: angular.z +는 좌회전. offset +는 차로 중심이 오른쪽(우조향 필요) → 부호 반전.
            cmd.angular.z = -(self.steer_gain * current_error) - (self.steer_d_gain * error_diff)
            
            # 4. 다음 프레임 연산을 위해 현재 오차를 저장
            self.prev_error = current_error
            
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
