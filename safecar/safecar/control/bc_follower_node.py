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
from safecar.control.mrm_profile import MrmProfile
from safecar.protocol import COMMAND_MRM_PULL_OVER, COMMAND_NORMAL


class BcFollowerNode(Node):
    """모방학습 모델로 조향한다 — lane_follower_node를 대신하는 주행 노드. (제어부)

    카메라 영상 → bc_model.preprocess → CNN(ONNX, OpenCV DNN으로 CPU 추론) → angular.z.
    속도는 cruise_speed로 고정한다(녹화할 때와 같은 속도로 달려야 조향값이 맞는다).
    lane_follower와 똑같이 '/cmd_vel_raw'로만 내보내므로 안전 게이트·AEB·워치독은 그대로다.

    카메라가 image_timeout 넘게 끊기면 정지한다.

    갓길 대피(MRM): 모델은 '오른쪽으로 붙어라'를 배우지 않았으므로 횡이동 없이
    **차로 안에서 감속 정지(자차로정차, UN R157 기본 MRM)** 한다. 조향은 모델이 계속 맡는다.
    해제는 NORMAL일 때만 — EMERGENCY_BRAKE가 끼었다 풀려도 재출발하지 않는다.
    """

    def __init__(self):
        super().__init__('bc_follower_node')
        self.declare_parameter('model_path', os.path.expanduser('~/bc_model.onnx'))
        # 녹화 때 조종한 속도와 같게. 0.12 미만은 정지 마찰 때문에 안 움직인다.
        self.declare_parameter('cruise_speed', 0.15)
        # 모델 출력에 곱하는 배율. 1.0 = 사람이 조종한 그대로. 흔들리면 ↓, 곡선에서 밀리면 ↑
        self.declare_parameter('steer_scale', 1.0)
        self.declare_parameter('max_steer', 1.0)        # rad/s 상한 (이상 출력 방어)
        self.declare_parameter('image_timeout', 0.5)
        self.declare_parameter('mrm_transition_time', 3.0)
        self.declare_parameter('mrm_speed_ratio', 0.85)  # 0.12 m/s 아래로 떨어지지 않게(lane_follower와 같은 근거)
        self.declare_parameter('mrm_stop_duration', 2.0)
        p = {n: self.get_parameter(n).value for n in (
            'model_path', 'cruise_speed', 'steer_scale', 'max_steer', 'image_timeout',
            'mrm_transition_time', 'mrm_speed_ratio', 'mrm_stop_duration')}
        self.cruise_speed = p['cruise_speed']
        self.steer_scale = p['steer_scale']
        self.max_steer = p['max_steer']
        self.image_timeout = p['image_timeout']
        self.mrm = MrmProfile(lateral_bias=0.0, transition_time=p['mrm_transition_time'],
                              speed_ratio=p['mrm_speed_ratio'], stop_duration=p['mrm_stop_duration'])

        if not os.path.exists(p['model_path']):
            raise SystemExit(f"모델 파일이 없다: {p['model_path']} (노트북에서 train_bc.py로 만든 뒤 scp)")
        self.net = cv2.dnn.readNetFromONNX(p['model_path'])
        self.get_logger().info(f"모델 로드: {p['model_path']}")

        self.bridge = CvBridge()
        self.steer = 0.0
        self.last_image_time = None
        self.driving = False
        self.mrm_start_time = None

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
        if msg.data == COMMAND_MRM_PULL_OVER and self.mrm_start_time is None:
            self.mrm_start_time = self.get_clock().now()
            self.get_logger().warn('운전자 이상 — MRM 시작 (자차로정차: 차로 안에서 감속 정지)')
        elif msg.data == COMMAND_NORMAL and self.mrm_start_time is not None:
            self.mrm_start_time = None
            self.get_logger().info('MRM 해제 — 정상 주행 복귀')

    def _on_timer(self):
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
            scale = 1.0
            if self.mrm_start_time is not None:
                scale, _ = self.mrm.compute((now - self.mrm_start_time).nanoseconds * 1e-9)
            cmd.linear.x = self.cruise_speed * scale
            cmd.angular.z = self.steer if scale > 0.0 else 0.0
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
