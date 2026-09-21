import json
import socket
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class SensorBridgeNode(Node):
    """외부 장치의 생체/운전자 상태 신호를 /sensors/bio_anomaly 로 중계한다. (통신부)

    source 파라미터로 입력원을 고른다. 어느 경우든 이 토픽의 발행자는 이 노드 하나뿐이다.

    - 'sim' (기본값): 기존 임시 시뮬레이션. anomaly_delay_sec 경과 시 이상 발생.
      anomaly_delay_sec을 0 이하로 주면 이상 신호를 발생시키지 않는다
      (teleop/주행 게이트 검증처럼 NORMAL 상태를 계속 유지해야 하는 테스트용).

    - 'udp': 운전자 졸음 감지 노트북(drowsy_v5.py)이 Wi-Fi로 보내는 UDP를 받는다.
      메시지(JSON): {"seq": int, "anomaly": bool, "state": str, "closed_dur": float}
      노트북은 초당 약 10회 보내며, anomaly는 노트북 쪽에서 래치된다
      (3초 이상 눈감김 시 True, 운전자가 해제할 때까지 True 유지).
      연결 끊김(udp_timeout_sec 동안 수신 없음) 시:
        * 마지막 값이 True였으면 True 유지 (갓길 정차 도중 재출발 방지)
        * 아니면 False 유지 + 경고 로그

    TODO: ESP32/STM32 생체센서 시리얼 연동 시 source='serial' 추가, 또는 udp 신호와 OR 결합.
    참고: 모터 제어는 이 노드가 아니라 STELLA N1의 기존 stella_md 노드가
    '/cmd_vel' 토픽으로 직접 담당한다 (시리얼 포트를 stella_md가 이미 점유함).
    """

    def __init__(self):
        super().__init__('sensor_bridge_node')
        self.declare_parameter('source', 'sim')
        self.declare_parameter('anomaly_delay_sec', 10.0)
        self.declare_parameter('udp_port', 5005)
        self.declare_parameter('udp_timeout_sec', 1.0)

        self.source = str(self.get_parameter('source').value)
        self.anomaly_delay_sec = float(self.get_parameter('anomaly_delay_sec').value)
        self.udp_port = int(self.get_parameter('udp_port').value)
        self.udp_timeout_sec = float(self.get_parameter('udp_timeout_sec').value)

        self.bio_anomaly_pub = self.create_publisher(Bool, '/sensors/bio_anomaly', 10)
        self.start_time = time.time()

        if self.source == 'udp':
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(('0.0.0.0', self.udp_port))
            self.sock.setblocking(False)     # 데이터가 없어도 기다리지 않고 바로 반환

            self.anomaly = False
            self.link_ok = False
            self.last_rx = None
            self.last_seq = None
            self.last_state = '-'
            self.rx_count = 0
            self.lost = 0
            self.last_status = time.time()

            self.get_logger().info(
                f'UDP 모드: {self.udp_port} 포트에서 졸음 감지 노트북 신호 대기 '
                f'(끊김 판정 {self.udp_timeout_sec:.1f}초)')
            self.create_timer(0.1, self._on_timer_udp)   # 10Hz (decision_maker와 같은 주기)
        else:
            self.get_logger().info(f'시뮬레이션 모드: anomaly_delay_sec={self.anomaly_delay_sec}')
            self.create_timer(0.5, self._on_timer_sim)

    # ================= 시뮬레이션 (기존 동작 그대로) =================
    def _on_timer_sim(self):
        msg = Bool()
        msg.data = (
            self.anomaly_delay_sec > 0
            and (time.time() - self.start_time) > self.anomaly_delay_sec
        )
        self.bio_anomaly_pub.publish(msg)

    # ================= UDP (졸음 감지 노트북) =================
    def _on_timer_udp(self):
        now = time.time()
        self._drain_socket(now)

        # 연결 끊김 감지
        if self.link_ok and now - self.last_rx > self.udp_timeout_sec:
            self.link_ok = False
            if self.anomaly:
                self.get_logger().error(
                    f'노트북 신호 {self.udp_timeout_sec:.1f}초 이상 끊김 '
                    f'- 이상 상태(True) 유지')
            else:
                self.get_logger().warn(
                    f'노트북 신호 {self.udp_timeout_sec:.1f}초 이상 끊김 '
                    f'- 졸음 감지 불가 상태')

        # 10초마다 수신 현황
        if now - self.last_status >= 10.0:
            self.get_logger().info(
                f'수신 {self.rx_count}개, 손실 추정 {self.lost}개, '
                f'연결 {"정상" if self.link_ok else "없음"}, anomaly={self.anomaly}')
            self.last_status = now

        msg = Bool()
        msg.data = bool(self.anomaly)
        self.bio_anomaly_pub.publish(msg)

    def _drain_socket(self, now):
        """쌓여 있는 UDP 패킷을 전부 읽는다. 마지막 패킷의 값이 최종값이 된다."""
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
            except BlockingIOError:
                return                      # 더 읽을 게 없음
            except OSError as e:
                self.get_logger().warn(f'UDP 수신 오류: {e}')
                return

            try:
                pkt = json.loads(data.decode('utf-8'))
                anomaly = bool(pkt['anomaly'])
            except (ValueError, KeyError, TypeError, UnicodeDecodeError):
                self.get_logger().warn(f'해석할 수 없는 UDP 데이터 무시: {data[:60]!r}')
                continue

            seq = pkt.get('seq')
            if isinstance(seq, int) and isinstance(self.last_seq, int) and seq > self.last_seq + 1:
                self.lost += seq - self.last_seq - 1
            self.last_seq = seq
            self.rx_count += 1
            self.last_rx = now
            self.last_state = pkt.get('state', '-')

            if not self.link_ok:
                self.link_ok = True
                self.get_logger().info(f'노트북 연결됨 <- {addr[0]}')

            if anomaly != self.anomaly:
                self.anomaly = anomaly
                text = (f'bio_anomaly = {anomaly} '
                        f'(state={self.last_state}, closed={pkt.get("closed_dur")}s)')
                if anomaly:
                    self.get_logger().warn(text)
                else:
                    self.get_logger().info(text)

    def destroy_node(self):
        if self.source == 'udp':
            self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
