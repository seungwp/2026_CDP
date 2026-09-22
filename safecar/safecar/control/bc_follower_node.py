import os

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from safecar.control.bc_model import preprocess
from safecar.protocol import COMMAND_NORMAL


class BcFollowerNode(Node):
    """모방학습 모델로 조향한다 — lane_follower_node를 대신하는 주행 노드. (제어부)

    카메라 영상 → bc_model.preprocess → CNN(ONNX, OpenCV DNN으로 CPU 추론) → angular.z.
    속도는 cruise_speed로 고정한다(녹화할 때와 같은 속도로 달려야 조향값이 맞는다).
    lane_follower와 똑같이 '/cmd_vel_raw'로만 내보내므로 안전 게이트·AEB·워치독은 그대로다.

    카메라가 image_timeout 넘게 끊기면 정지한다.

    갓길 대피(MRM)는 이 노드가 하지 않는다 — 모델은 '오른쪽으로 붙어라'를 배우지 않았고,
    같은 영상(흰 선을 보는 장면)이 평소엔 '흰선 따라가라', MRM 중엔 '노란선 찾아 붙어라'로
    다른 답을 내야 하는데 지금 입력(영상만)으로는 그 둘을 구분할 방법이 없다(로타리에서
    같은 장면에 답이 여럿이라 헷갈리던 것과 같은 함정). 그래서 '/control/driving_state'가
    NORMAL이 아니게 되면 그냥 조용해진다(발행을 멈춘다) — lane_follower_node를
    drive_normal:=false로 같이 띄워두면 그쪽이 이어받아 노란 갓길선을 실제로 추종·정차시킨다
    (safecar_drive.sh bc가 이미 그렇게 띄운다). 혹시 lane_follower가 안 떠 있어도
    decision_maker의 cmd_vel_timeout(1초)이 최후 안전망으로 정지시킨다.
    """

    def __init__(self):
        super().__init__('bc_follower_node')
        self.declare_parameter('model_path', os.path.expanduser('~/bc_model.onnx'))
        # **녹화 때 조종한 속도와 같아야 한다** — 모델은 조향을 각속도(rad/s)로 배우는데
        # 각속도 = 속도 × 곡률이라, 다른 속도로 달리면 같은 조향값이 다른 궤적을 그린다.
        # 0.3 = 현재 학습 데이터(dataset/20260922_131811)의 녹화 속도.
        # 다른 속도로 달리려면 steer_scale에 속도비를 준다(예: 0.15면 0.5).
        # 하한: 0.12 미만은 정지 마찰 때문에 안 움직인다.
        self.declare_parameter('cruise_speed', 0.3)
        # 모델 출력에 곱하는 배율. 1.0 = 사람이 조종한 그대로. 흔들리면 ↓, 곡선에서 밀리면 ↑
        self.declare_parameter('steer_scale', 1.0)
        self.declare_parameter('max_steer', 1.0)        # rad/s 상한 (이상 출력 방어)
        self.declare_parameter('image_timeout', 0.5)
        p = {n: self.get_parameter(n).value for n in (
            'model_path', 'cruise_speed', 'steer_scale', 'max_steer', 'image_timeout')}
        self.cruise_speed = p['cruise_speed']
        self.steer_scale = p['steer_scale']
        self.max_steer = p['max_steer']
        self.image_timeout = p['image_timeout']

        if not os.path.exists(p['model_path']):
            raise SystemExit(f"모델 파일이 없다: {p['model_path']} (노트북에서 train_bc.py로 만든 뒤 scp)")
        self.net = cv2.dnn.readNetFromONNX(p['model_path'])
        self.get_logger().info(f"모델 로드: {p['model_path']}")

        self.bridge = CvBridge()
        self.steer = 0.0
        self.last_image_time = None
        self.driving = False
        # driving_state 토픽이 아직 안 왔으면(decision_maker 없이 단독 테스트 등)
        # 평소처럼 운전하는 쪽을 기본값으로 한다.
        self.active = True

        self.create_subscription(Image, '/camera/image_raw', self._on_image, qos_profile_sensor_data)
        self.create_subscription(String, '/control/driving_state', self._on_state, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.create_timer(0.05, self._on_timer)  # 20Hz

    def _on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.net.setInput(preprocess(frame)[None])
        raw = float(self.net.forward().reshape(-1)[0])
        self.steer = max(-self.max_steer, min(self.max_steer, raw * self.steer_scale))
        self.last_image_time = self.get_clock().now()

    def _on_state(self, msg):
        active = (msg.data == COMMAND_NORMAL)
        if active != self.active:
            self.active = active
            self.get_logger().info(
                '정상 주행 복귀 — 계속 운전' if active else
                '운전자 이상/장애물 — 발행 중단 (lane_follower가 이어받음)')

    def _on_timer(self):
        if not self.active:
            return  # NORMAL이 아니면 아무것도 안 낸다 — MRM은 lane_follower_node가 맡는다

        now = self.get_clock().now()
        fresh = (self.last_image_time is not None
                 and (now - self.last_image_time).nanoseconds * 1e-9 < self.image_timeout)
        if fresh != self.driving:
            if fresh:
                self.get_logger().info('카메라 수신 — 주행 시작')
            else:
                self.get_logger().warn(f'카메라 {self.image_timeout:.1f}초 이상 끊김 — 정지')
            self.driving = fresh

        cmd = Twist()
        if fresh:
            cmd.linear.x = self.cruise_speed
            cmd.angular.z = self.steer
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BcFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
