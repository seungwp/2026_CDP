"""차량 주행 상태를 차량용 ESP32(A)로 넘겨 주변 차량에 V2V 방송하게 한다. (통신부)

    /control/driving_state (String) ─┐
                                     ├─▶ USB 시리얼 "S,<code>,<speed>\\n" (10Hz) ─▶ ESP32-A ─ESP-NOW─▶ 뒤차 화면
    /odom (Odometry)  ───────────────┘
                                     ◀─ "A,..." 상태 보고 (1Hz) ─▶ /v2x/status (String)

위험코드 (docs/V2X_SIGNAL_CONTRACT.md)
    0 = NORMAL           정상
    1 = EMERGENCY_BRAKE  전방 장애물 급제동
    2 = MRM_PULL_OVER    운전자 이상 갓길 비상정차
    3 = 상태 모름        알 수 없는 상태값 / driving_state가 state_timeout_sec 동안 안 옴

안전 동작
    - decision_maker가 멈추면(상태 수신 끊김) 코드 3 -> 뒤차 화면 "신호없음"
    - 이 노드가 멈추면 ESP32-A가 1초 뒤 스스로 코드 3 방송 (펌웨어 쪽 하트비트)
    - ESP32가 안 꽂혀 있거나 빠지면 노드는 죽지 않고 reconnect_sec마다 다시 연다
      (통합 launch에 들어가는 노드라, 이것 때문에 다른 노드가 영향받으면 안 된다)

포트는 번호(/dev/ttyUSB0)가 아니라 udev 고정 이름(/dev/esp32_v2x)을 쓴다.
YDLIDAR X4도 같은 CP210x 칩이라 번호는 꽂는 순서에 따라 바뀐다.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

try:
    import serial
except ImportError:          # python3-serial 미설치 시 노드는 뜨되 이유를 알려준다
    serial = None

from safecar.protocol import (
    COMMAND_NORMAL,
    COMMAND_EMERGENCY_BRAKE,
    COMMAND_MRM_PULL_OVER,
)

CODE_UNKNOWN = 3
STATE_TO_CODE = {
    COMMAND_NORMAL: 0,
    COMMAND_EMERGENCY_BRAKE: 1,
    COMMAND_MRM_PULL_OVER: 2,
}
CODE_NAME = {0: '정상', 1: '급제동', 2: '비상정차', 3: '상태모름'}


class V2xBridgeNode(Node):

    def __init__(self):
        super().__init__('v2x_bridge_node')
        self.declare_parameter('port', '/dev/esp32_v2x')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('state_timeout_sec', 1.0)
        self.declare_parameter('odom_timeout_sec', 1.0)
        self.declare_parameter('reconnect_sec', 2.0)

        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        rate_hz = float(self.get_parameter('rate_hz').value)
        self.state_timeout = float(self.get_parameter('state_timeout_sec').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout_sec').value)
        self.reconnect_sec = float(self.get_parameter('reconnect_sec').value)

        self.state = None
        self.state_time = None
        self.speed = 0.0
        self.odom_time = None

        self.ser = None
        self.last_open_try = 0.0
        self.rx_buf = b''
        self.last_code = None
        self.esp_link = None
        self.sent = 0
        self.warned_unknown = set()

        self.create_subscription(String, '/control/driving_state', self._on_state, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.status_pub = self.create_publisher(String, '/v2x/status', 10)

        if serial is None:
            self.get_logger().error('python3-serial이 없습니다: sudo apt install python3-serial')
        self.get_logger().info(f'V2X 브릿지: {self.port} @ {self.baud}bps, {rate_hz:g}Hz')
        self.create_timer(1.0 / rate_hz, self._on_timer)

    # ---------------- 입력 ----------------
    def _on_state(self, msg):
        self.state = msg.data
        self.state_time = time.time()

    def _on_odom(self, msg):
        self.speed = float(msg.twist.twist.linear.x)
        self.odom_time = time.time()

    def _current_code(self, now):
        if self.state_time is None or now - self.state_time > self.state_timeout:
            return CODE_UNKNOWN, '주행 상태 수신 없음'
        code = STATE_TO_CODE.get(self.state)
        if code is None:
            if self.state not in self.warned_unknown:      # 같은 경고 반복 방지
                self.warned_unknown.add(self.state)
                self.get_logger().warn(f'모르는 주행 상태 "{self.state}" -> 코드 3으로 방송')
            return CODE_UNKNOWN, f'모르는 상태 {self.state}'
        return code, self.state

    def _current_speed(self, now):
        if self.odom_time is None or now - self.odom_time > self.odom_timeout:
            return 0.0
        return max(-10.0, min(10.0, self.speed))   # ESP32 펌웨어 허용 범위

    # ---------------- 시리얼 ----------------
    def _ensure_open(self, now):
        if self.ser is not None or serial is None:
            return self.ser is not None
        if now - self.last_open_try < self.reconnect_sec:
            return False
        self.last_open_try = now
        try:
            s = serial.Serial()
            s.port = self.port
            s.baudrate = self.baud
            s.timeout = 0
            s.write_timeout = 0.2
            s.dtr = False
            s.rts = False
            s.open()
            self.ser = s
            self.rx_buf = b''
            self.get_logger().info(f'ESP32-A 연결: {self.port}')
            return True
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(
                f'ESP32-A를 열 수 없음 ({self.port}): {e} - {self.reconnect_sec:g}초 뒤 재시도',
                throttle_duration_sec=10.0)
            return False

    def _close(self, reason):
        self.get_logger().error(f'ESP32-A 연결 끊김: {reason}')
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.esp_link = None

    def _read_reports(self):
        try:
            data = self.ser.read(self.ser.in_waiting or 0)
        except (serial.SerialException, OSError) as e:
            self._close(e)
            return
        if not data:
            return
        self.rx_buf += data
        if len(self.rx_buf) > 4096:                    # 줄바꿈 없는 쓰레기가 쌓이면 버림
            self.rx_buf = self.rx_buf[-512:]
        while b'\n' in self.rx_buf:
            raw, self.rx_buf = self.rx_buf.split(b'\n', 1)
            line = raw.decode('utf-8', errors='replace').strip()
            if line.startswith('A,'):
                self._on_report(line)
            elif line.startswith('#'):
                self.get_logger().info(f'[ESP32-A] {line[1:].strip()}')
            # 그 외(부팅 메시지, 깨진 조각)는 무시

    def _on_report(self, line):
        f = line.split(',')
        if len(f) != 7:
            return
        msg = String()
        msg.data = line
        self.status_pub.publish(msg)
        link = f[1] == '1'
        if link != self.esp_link:
            if link:
                self.get_logger().info('ESP32-A: 브릿지 신호 수신 중 (방송 정상)')
            else:
                self.get_logger().warn('ESP32-A: 브릿지 신호 못 받음 -> 코드 3 방송 중')
            self.esp_link = link

    # ---------------- 주기 전송 ----------------
    def _on_timer(self):
        now = time.time()
        code, why = self._current_code(now)
        speed = self._current_speed(now)

        if code != self.last_code:
            prev = '-' if self.last_code is None else self.last_code
            self.get_logger().info(f'V2X 방송 코드 {prev} -> {code} ({CODE_NAME[code]}, {why})')
            self.last_code = code

        if not self._ensure_open(now):
            return
        try:
            self.ser.write(f'S,{code},{speed:.2f}\n'.encode('ascii'))
            self.sent += 1
        except (serial.SerialException, OSError) as e:
            self._close(e)
            return
        self._read_reports()

    def destroy_node(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = V2xBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
