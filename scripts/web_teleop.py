#!/usr/bin/env python3
"""휴대폰 브라우저 조종기 — 모방학습 데이터 녹화용 아날로그 조향.

키보드 teleop은 조향이 계단식이라 라벨 품질이 나쁘다. 이 스크립트는 휴대폰 화면을
조이스틱처럼 쓴다: **누르고 있는 동안 고정 속도로 전진, 손가락 좌우 위치만큼 조향.**
손을 떼면 즉시 정지. (DonkeyCar의 웹 컨트롤러와 같은 방식)

사용 (Pi):
    ros2 launch safecar manual_drive.launch.py          # 터미널 1
    python3 ~/2026_CDP/scripts/web_teleop.py            # 터미널 2
    python3 ~/2026_CDP/scripts/record_dataset.py        # 터미널 3 (녹화)
휴대폰 (Pi와 같은 핫스팟):  http://raspberrypi.local:8000/

    --speed 0.15      전진 속도(m/s). 모방학습 주행 속도와 같게 둘 것
    --max-steer 0.8   화면 끝까지 밀었을 때 angular.z (rad/s)
    --topic /cmd_vel  manual_drive(게이트 없음)용. safecar_start(게이트 있음)와 쓰면 /cmd_vel_raw

안전:
- 휴대폰에서 0.3초 넘게 신호가 안 오면(손 뗌, Wi-Fi 끊김, 탭 닫힘) 정지 명령을 보낸다.
- 정지 후에는 **아무것도 발행하지 않는다.** 계속 0을 보내면 나중에 켠 안전 게이트와
  /cmd_vel을 두고 싸운다. 발행이 멈추면 stella_md 워치독(0.5초)이 모터를 세운다.

로직만 확인하려면:  python3 scripts/web_teleop.py --selftest
"""

import argparse
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

PAGE = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<meta name="apple-mobile-web-app-capable" content="yes">
<title>SafeCar 조종</title>
<style>
/* iOS Safari: 100vh는 주소창까지 포함해 아래가 잘린다 → 100dvh(실제 보이는 높이).
   다이내믹 아일랜드·홈 바는 safe-area로 비켜 간다. 길게 누를 때 선택/돋보기/튕김도 막는다. */
*{box-sizing:border-box;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none;
  -webkit-tap-highlight-color:transparent}
html,body{margin:0;height:100%;background:#111;color:#eee;font-family:-apple-system,sans-serif;
  overflow:hidden;overscroll-behavior:none;position:fixed;inset:0}
body{display:flex;flex-direction:column;height:100dvh;
  padding:env(safe-area-inset-top) env(safe-area-inset-right) env(safe-area-inset-bottom) env(safe-area-inset-left)}
#cam{display:block;width:100%;max-height:35dvh;object-fit:contain;background:#000;flex:none}
#pad{position:relative;flex:1;min-height:0;touch-action:none;border-radius:12px;margin:6px;
     background:linear-gradient(90deg,#1c2a3a,#222 50%,#1c2a3a)}
#mid{position:absolute;left:50%;top:0;bottom:0;border-left:2px dashed #555}
#dot{position:absolute;top:50%;width:64px;height:64px;margin:-32px;border-radius:50%;
     background:#555;left:50%;transition:background .1s;pointer-events:none}
#info{position:absolute;top:10px;width:100%;text-align:center;font-size:18px;pointer-events:none}
</style></head><body>
<img id="cam" src="" alt="" draggable="false">
<div id="pad"><div id="mid"></div><div id="dot"></div>
<div id="info">누르고 있으면 전진 · 좌우로 조향 · 떼면 정지</div></div>
<script>
const pad=document.getElementById('pad'),dot=document.getElementById('dot'),info=document.getElementById('info');
const cam=document.getElementById('cam');
cam.src='http://'+location.hostname+':{cam_port}/stream';
cam.onerror=()=>{cam.style.display='none'};
document.addEventListener('touchmove',e=>e.preventDefault(),{passive:false});  // 튕김·스크롤 방지
let steer=0,down=false,timer=null;
function send(go){fetch('/cmd?go='+(go?1:0)+'&steer='+steer.toFixed(3)).catch(()=>{});}
function pos(e){const r=pad.getBoundingClientRect();
  steer=Math.max(-1,Math.min(1,((e.clientX-r.left)/r.width)*2-1));
  dot.style.left=((steer+1)/2*100)+'%';info.textContent='조향 '+steer.toFixed(2);}
pad.addEventListener('pointerdown',e=>{pad.setPointerCapture(e.pointerId);down=true;pos(e);
  dot.style.background='#2e8b57';send(true);timer=setInterval(()=>send(true),50);});
pad.addEventListener('pointermove',e=>{if(down)pos(e);});
function up(){if(!down)return;down=false;clearInterval(timer);steer=0;send(false);
  dot.style.left='50%';dot.style.background='#555';info.textContent='정지';}
pad.addEventListener('pointerup',up);pad.addEventListener('pointercancel',up);
document.addEventListener('visibilitychange',()=>{if(document.hidden)up();});
// 손 뗌 이벤트가 누락돼도 멈추게: 알림·제어 센터로 포커스를 잃거나 페이지를 떠나면 정지
window.addEventListener('blur',up);window.addEventListener('pagehide',up);
document.addEventListener('touchend',e=>{if(e.touches.length===0)up();});
document.addEventListener('touchcancel',up);
</script></body></html>"""


def command(go, steer, age, speed, max_steer, timeout):
    """휴대폰 입력 → (linear_x, angular_z). 신호가 오래됐거나 go=False면 정지.

    steer는 화면 위치 -1(왼쪽 끝)~+1(오른쪽 끝). REP-103에서 angular.z +는 좌회전이므로
    오른쪽으로 밀면 음수가 되도록 부호를 뒤집는다.
    """
    if not go or age > timeout:
        return 0.0, 0.0
    steer = max(-1.0, min(1.0, steer))
    return speed, -steer * max_steer


def _self_check():
    base = dict(speed=0.15, max_steer=0.8, timeout=0.3)
    assert command(True, 0.0, 0.0, **base) == (0.15, 0.0)          # 가운데 = 직진
    assert command(True, 1.0, 0.0, **base) == (0.15, -0.8)         # 오른쪽 끝 = 우회전(음수)
    assert command(True, -0.5, 0.0, **base) == (0.15, 0.4)         # 왼쪽 = 좌회전(양수)
    assert command(True, 5.0, 0.0, **base) == (0.15, -0.8)         # 범위 밖 입력은 잘라낸다
    assert command(False, 0.7, 0.0, **base) == (0.0, 0.0)          # 손 뗌 = 정지
    assert command(True, 0.7, 0.5, **base) == (0.0, 0.0)           # 신호 끊김 = 정지
    print('web_teleop self-check OK')


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node

    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8000)
    ap.add_argument('--cam-port', type=int, default=8081, help='image_server.py 원본 영상 포트(없으면 영상 숨김)')
    ap.add_argument('--speed', type=float, default=0.15)
    ap.add_argument('--max-steer', type=float, default=0.8)
    ap.add_argument('--timeout', type=float, default=0.3)
    ap.add_argument('--topic', default='/cmd_vel')
    args = ap.parse_args()

    state = {'go': False, 'steer': 0.0, 't': 0.0}
    lock = threading.Lock()
    page = PAGE.replace('{cam_port}', str(args.cam_port)).encode('utf-8')

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            url = urlparse(self.path)
            if url.path == '/cmd':
                q = parse_qs(url.query)
                try:
                    go = q.get('go', ['0'])[0] == '1'
                    steer = float(q.get('steer', ['0'])[0])
                except ValueError:
                    go, steer = False, 0.0
                with lock:
                    state.update(go=go, steer=steer, t=time.monotonic())
                self.send_response(204)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(page)))
            self.end_headers()
            self.wfile.write(page)

    class WebTeleop(Node):
        def __init__(self):
            super().__init__('web_teleop')
            self.pub = self.create_publisher(Twist, args.topic, 10)
            self.stop_left = 0   # 정지 후 몇 번 더 0을 보낼지
            self.create_timer(0.05, self._tick)  # 20Hz

        def _tick(self):
            with lock:
                go, steer, t = state['go'], state['steer'], state['t']
            lin, ang = command(go, steer, time.monotonic() - t,
                               args.speed, args.max_steer, args.timeout)
            if lin == 0.0:
                if self.stop_left <= 0:
                    return  # 정지 상태에서는 발행하지 않는다(게이트와 싸우지 않게)
                self.stop_left -= 1
            else:
                self.stop_left = 5
            msg = Twist()
            msg.linear.x, msg.angular.z = lin, ang
            self.pub.publish(msg)

    rclpy.init()
    node = WebTeleop()
    server = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(
        f'휴대폰에서 http://raspberrypi.local:{args.port}/ 접속 — {args.topic}, '
        f'속도 {args.speed}, 최대 조향 {args.max_steer}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    if '--selftest' in sys.argv:
        _self_check()
    else:
        main()
