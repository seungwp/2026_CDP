#!/usr/bin/env python3
"""ROS 이미지 토픽을 브라우저로 띄운다 — VM/rqt 없이 노트북에서 카메라를 보기 위한 것.

카메라를 물리적으로 조준하거나 차선 검출 상수를 튜닝할 때, 노트북 브라우저에서
    http://<라즈베리파이IP>:8080/
를 열면 실시간 영상이 나온다. 우분투 VM도 rqt도 필요 없다.

  /            라이브 영상 페이지
  /stream      MJPEG 스트림
  /snapshot    현재 프레임 1장 (JPEG) — 튜닝 근거 이미지를 파일로 받을 때

사용:
    # 터미널 1 — 카메라(+차선인식)를 먼저 띄운다
    ros2 launch safecar safecar.launch.py lane_follow:=true

    # 터미널 2 — 스트리머 (환경 소싱은 record_drive.sh와 동일)
    python3 scripts/image_server.py                            # 기본: /perception/lane_image
    python3 scripts/image_server.py --topic /camera/image_raw  # 원본 영상(카메라 조준용)
    python3 scripts/image_server.py --port 8080 --quality 70

주의: camera_node는 SIGTERM에서 멈추지 않고 CSI를 물고 있는 버그가 있어
      종료할 때 반드시 `pkill -9 -f camera_node` 로 정리해야 다음 실행이 된다.
"""

import argparse
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

PAGE = """<!doctype html><meta charset="utf-8"><title>SafeCar camera</title>
<style>body{{margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center}}
img{{max-width:100%;height:auto;margin-top:8px}}p{{font-size:14px;color:#aaa}}</style>
<p>{topic} &nbsp;|&nbsp; <a style="color:#6af" href="/snapshot">스냅샷 저장</a></p>
<img src="/stream">
"""


class Latest:
    """최신 프레임 1장만 들고 있는 공유 슬롯 (구독 스레드 → HTTP 스레드)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._jpeg = None
        self._seq = 0
        self._new = threading.Condition(self._lock)

    def put(self, jpeg):
        with self._lock:
            self._jpeg = jpeg
            self._seq += 1
            self._new.notify_all()

    def get(self):
        with self._lock:
            return self._jpeg, self._seq

    def wait_next(self, last_seq, timeout=5.0):
        """마지막으로 보낸 것보다 새 프레임이 나올 때까지 기다린다(없으면 None)."""
        with self._lock:
            if self._seq == last_seq:
                self._new.wait(timeout)
            return self._jpeg, self._seq


def make_handler(latest, topic):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass  # 요청마다 콘솔 더럽히지 않는다

        def do_GET(self):
            if self.path.startswith('/stream'):
                self._stream()
            elif self.path.startswith('/snapshot'):
                self._snapshot()
            else:
                body = PAGE.format(topic=topic).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        def _snapshot(self):
            jpeg, _ = latest.get()
            if jpeg is None:
                self.send_error(503, 'no frame yet')
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(jpeg)))
            self.send_header('Content-Disposition', 'attachment; filename="frame.jpg"')
            self.end_headers()
            self.wfile.write(jpeg)

        def _stream(self):
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            seq = -1
            try:
                while True:
                    jpeg, seq = latest.wait_next(seq)
                    if jpeg is None:
                        continue
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                     b'Content-Length: ' + str(len(jpeg)).encode()
                                     + b'\r\n\r\n' + jpeg + b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass  # 브라우저 탭 닫힘 — 정상

    return Handler


class ImageServerNode(Node):
    def __init__(self, latest, topic, quality):
        super().__init__('image_server')
        self.latest = latest
        self.bridge = CvBridge()
        self.params = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        self.count = 0
        self.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)
        self.get_logger().info(f"'{topic}' 구독 중 — 첫 프레임 대기")

    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패: {e}')
            return
        ok, buf = cv2.imencode('.jpg', frame, self.params)
        if ok:
            self.latest.put(buf.tobytes())
            self.count += 1
            if self.count == 1:
                h, w = frame.shape[:2]
                self.get_logger().info(f'첫 프레임 수신 {w}x{h} — 브라우저에서 접속하세요')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/perception/lane_image')
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--quality', type=int, default=80)
    args = ap.parse_args()

    latest = Latest()
    rclpy.init()
    node = ImageServerNode(latest, args.topic, args.quality)

    server = ThreadingHTTPServer(('0.0.0.0', args.port), make_handler(latest, args.topic))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f'http://<이 장비 IP>:{args.port}/ 에서 볼 수 있습니다')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
