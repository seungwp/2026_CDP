#!/usr/bin/env python3
"""모방학습용 데이터 녹화 — 사람이 조종하는 동안 (카메라 영상, 조향값) 짝을 저장한다.

라벨은 '/cmd_vel'(실제로 모터에 들어간 명령)에서 가져온다. 그래서 조종 장치가
키보드든 조이스틱이든, 수동 주행 launch든 통합 launch(게이트 경유)든 상관없다.

저장 형식 (세션마다 새 폴더):
    ~/dataset/20260922_143000/
        images/000001.jpg ...     320x240 JPEG
        labels.csv                frame,stamp,linear_x,angular_z

- 멈춰 있는 동안(linear_x ≈ 0)은 저장하지 않는다 — 정지 장면은 조향 학습에 쓸모가 없고
  데이터만 치우치게 만든다. (--keep-stopped 로 끌 수 있음)
- 조향 명령이 cmd_timeout 넘게 안 들어왔으면 저장하지 않는다 — 라벨이 오래된 값이다.
- 한 줄씩 바로 기록하고 주기적으로 fsync 한다. Pi 전원이 불안해서(배터리 마진 0)
  도중에 꺼져도 그때까지의 데이터는 남아야 한다.

사용:
    # 터미널 1 — 차체 + 카메라
    ros2 launch safecar manual_drive.launch.py
    # 터미널 2 — 조종 (휴대폰 브라우저 http://raspberrypi.local:8000/ , 아날로그 조향)
    python3 ~/2026_CDP/scripts/web_teleop.py
    # 터미널 3 — 녹화 (Ctrl+C로 종료)
    source ~/safecar_env.sh
    python3 ~/2026_CDP/scripts/record_dataset.py
    python3 ~/2026_CDP/scripts/record_dataset.py --rate 15 --out ~/dataset

⚠️ 복구 데이터를 꼭 섞을 것: 차선 중앙만 달린 데이터로 학습하면, 모델이 조금만
    벗어나도 처음 보는 장면이 돼서 그대로 이탈한다. 일부러 옆으로 비켰다가
    돌아오는 구간을 전체의 20~30% 정도 녹화한다.

로직만 확인하려면:  python3 scripts/record_dataset.py --selftest
"""

import argparse
import os
import sys
import time


def should_save(now, last_saved, cmd_time, linear_x,
                period, cmd_timeout, min_speed, keep_stopped=False):
    """이 프레임을 저장할지. (저장 여부, 건너뛴 이유)"""
    if cmd_time is None:
        return False, '조향 명령 없음'
    if now - cmd_time > cmd_timeout:
        return False, '조향 명령 끊김'
    if not keep_stopped and abs(linear_x) < min_speed:
        return False, '정지 중'
    # 1ms 여유: 카메라 주기가 저장 주기와 거의 같을 때 부동소수점 오차로 한 장씩 건너뛰지 않게
    if last_saved is not None and now - last_saved < period - 1e-3:
        return False, '주기 제한'
    return True, ''


def _self_check():
    base = dict(period=0.1, cmd_timeout=0.3, min_speed=0.01)

    # 명령이 없거나 끊기면 저장 안 함
    assert should_save(10.0, None, None, 0.15, **base) == (False, '조향 명령 없음')
    assert should_save(10.0, None, 9.5, 0.15, **base) == (False, '조향 명령 끊김')
    # 정상 주행 중 첫 프레임은 저장
    assert should_save(10.0, None, 9.9, 0.15, **base) == (True, '')
    # 멈춰 있으면 저장 안 함, 옵션으로 켜면 저장
    assert should_save(10.0, None, 9.9, 0.0, **base) == (False, '정지 중')
    assert should_save(10.0, None, 9.9, 0.0, keep_stopped=True, **base)[0] is True
    # 후진도 움직이는 것으로 본다
    assert should_save(10.0, None, 9.9, -0.15, **base)[0] is True
    # 주기 제한: 0.1초 안에 들어온 다음 프레임은 건너뜀
    assert should_save(10.05, 10.0, 10.0, 0.15, **base) == (False, '주기 제한')
    assert should_save(10.1, 10.0, 10.0, 0.15, **base)[0] is True
    print('record_dataset self-check OK')


def main():
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.expanduser('~/dataset'))
    ap.add_argument('--rate', type=float, default=10.0, help='최대 저장 주기(Hz)')
    ap.add_argument('--width', type=int, default=320)
    ap.add_argument('--height', type=int, default=240)
    ap.add_argument('--quality', type=int, default=90)
    ap.add_argument('--cmd-timeout', type=float, default=0.3)
    ap.add_argument('--min-speed', type=float, default=0.01)
    ap.add_argument('--keep-stopped', action='store_true')
    args = ap.parse_args()

    session = os.path.join(args.out, time.strftime('%Y%m%d_%H%M%S'))
    img_dir = os.path.join(session, 'images')
    os.makedirs(img_dir)

    class Recorder(Node):
        def __init__(self):
            super().__init__('record_dataset')
            self.bridge = CvBridge()
            self.jpeg = [int(cv2.IMWRITE_JPEG_QUALITY), args.quality]
            self.csv = open(os.path.join(session, 'labels.csv'), 'w', buffering=1)
            self.csv.write('frame,stamp,linear_x,angular_z\n')
            self.cmd = None
            self.cmd_time = None
            self.last_saved = None
            self.saved = 0
            self.skips = {}
            self.create_subscription(Image, '/camera/image_raw', self._on_image,
                                     qos_profile_sensor_data)
            self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
            self.create_timer(5.0, self._report)
            self.get_logger().info(f'녹화 폴더: {session}')

        def _on_cmd(self, msg):
            self.cmd = msg
            self.cmd_time = time.monotonic()

        def _on_image(self, msg):
            now = time.monotonic()
            lin = self.cmd.linear.x if self.cmd else 0.0
            ok, why = should_save(now, self.last_saved, self.cmd_time, lin,
                                  1.0 / args.rate, args.cmd_timeout, args.min_speed,
                                  args.keep_stopped)
            if not ok:
                self.skips[why] = self.skips.get(why, 0) + 1
                return
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.resize(frame, (args.width, args.height), interpolation=cv2.INTER_AREA)
            self.saved += 1
            name = f'{self.saved:06d}.jpg'
            cv2.imwrite(os.path.join(img_dir, name), frame, self.jpeg)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.csv.write(f'{name},{stamp:.3f},{lin:.4f},{self.cmd.angular.z:.4f}\n')
            if self.saved % 50 == 0:
                os.fsync(self.csv.fileno())  # 전원이 나가도 여기까지는 남긴다
            self.last_saved = now

        def _report(self):
            steer = f'{self.cmd.angular.z:+.2f}' if self.cmd else '-'
            skips = ', '.join(f'{k} {v}' for k, v in self.skips.items() if k != '주기 제한')
            print(f'[녹화] 저장 {self.saved}장  현재 조향 {steer}' + (f'  (건너뜀: {skips})' if skips else ''))
            self.skips = {}

        def close(self):
            self.csv.flush()
            os.fsync(self.csv.fileno())
            self.csv.close()

    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        print(f'\n종료 — {node.saved}장 저장: {session}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    if '--selftest' in sys.argv:
        _self_check()
    else:
        main()
