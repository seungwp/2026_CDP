#!/usr/bin/env python3
"""휴대폰 브라우저 조종기 — 모방학습 데이터 녹화용 아날로그 조향.

키보드 teleop은 조향이 계단식이라 라벨 품질이 나쁘다. 이 스크립트는 휴대폰 화면을
조이스틱처럼 쓴다: **누르고 있는 동안 고정 속도로 전진, 손가락 좌우 위치만큼 조향.**
손을 떼면 즉시 정지. (DonkeyCar의 웹 컨트롤러와 같은 방식)

화면 위에 Pi 연결 상태를 표시한다(끊기면 빨간 경고).

사용 (Pi):
    ros2 launch safecar manual_drive.launch.py          # 터미널 1
    python3 ~/2026_CDP/scripts/web_teleop.py            # 터미널 2
    python3 ~/2026_CDP/scripts/record_dataset.py        # 터미널 3 (녹화)
휴대폰 (Pi와 같은 핫스팟):  http://raspberrypi.local:8000/

    --speed 0.15      전진 속도(m/s). 모방학습 주행 속도와 같게 둘 것
    --max-steer 0.8   화면 끝까지 밀었을 때 angular.z (rad/s)
    --topic /cmd_vel  조종 명령 토픽 (manual_drive용)

안전:
- 휴대폰에서 0.3초 넘게 신호가 안 오면(손 뗌, Wi-Fi 끊김, 탭 닫힘) 정지 명령을 보낸다.
- HTTP 요청은 순서가 뒤바뀌거나 폰에 밀려 쌓일 수 있다. 순번으로 옛 요청을 버리고,
  앞 요청이 끝나야 다음 전진을 보낸다(손을 뗐는데 밀린 '전진'이 도착해 안 멈추던 문제).
- 조종하지 않을 때는 조종 토픽에 **아무것도 발행하지 않는다**(안전 게이트와 싸우지 않게).
  발행이 멈추면 stella_md 워치독(0.5초)이 모터를 세운다.

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
#link{flex:none;text-align:center;font-size:14px;padding:4px;color:#8bc34a}
#link.bad{color:#fff;background:#b71c1c;font-size:18px}
#cam{display:block;width:100%;max-height:30dvh;object-fit:contain;background:#000;flex:none}
#pad{position:relative;flex:1;min-height:0;touch-action:none;border-radius:12px;margin:6px;
     background:linear-gradient(90deg,#1c2a3a,#222 50%,#1c2a3a)}
#mid{position:absolute;left:50%;top:0;bottom:0;border-left:2px dashed #555}
#dot{position:absolute;top:50%;width:64px;height:64px;margin:-32px;border-radius:50%;
     background:#555;left:50%;transition:background .1s;pointer-events:none}
#info{position:absolute;top:10px;width:100%;text-align:center;font-size:18px;pointer-events:none}
</style></head><body>
<div id="link">연결 확인 중...</div>
<img id="cam" src="" alt="" draggable="false">
<div id="pad"><div id="mid"></div><div id="dot"></div>
<div id="info">누르고 있으면 전진 · 좌우로 조향 · 떼면 정지</div></div>
<script>
const $=id=>document.getElementById(id);
const pad=$('pad'),dot=$('dot'),info=$('info'),cam=$('cam'),link=$('link');
cam.src='http://'+location.hostname+':{cam_port}/stream';
cam.onerror=()=>{cam.style.display='none'};
document.addEventListener('touchmove',e=>e.preventDefault(),{passive:false});  // 튕김·스크롤 방지

let steer=0,down=false,timer=null,seq=0,inflight=false;
const sid=Math.random().toString(36).slice(2);   // 탭마다 다른 id — 새로고침하면 순번을 새로 센다
// 전진 요청은 앞 요청이 끝나야 다음을 보낸다. 안 그러면 Wi-Fi가 느릴 때 요청이 폰에 쌓이고,
// 손을 뗀 뒤에도 밀린 '전진'이 몇 초씩 도착해 차가 안 멈춘다(실제로 겪음). 정지는 항상 즉시 보낸다.
function send(go){
  if(go&&inflight)return;
  inflight=go;seq++;
  fetch('/cmd?go='+(go?1:0)+'&steer='+steer.toFixed(3)+'&sid='+sid+'&seq='+seq,{cache:'no-store'})
    .catch(()=>{}).finally(()=>{if(go)inflight=false;});
}
function pos(e){const r=pad.getBoundingClientRect();
  steer=Math.max(-1,Math.min(1,((e.clientX-r.left)/r.width)*2-1));
  dot.style.left=((steer+1)/2*100)+'%';info.textContent='조향 '+steer.toFixed(2);}
pad.addEventListener('pointerdown',e=>{pad.setPointerCapture(e.pointerId);down=true;pos(e);
  dot.style.background='#2e8b57';send(true);timer=setInterval(()=>send(true),50);});
pad.addEventListener('pointermove',e=>{if(down)pos(e);});
function up(){if(!down)return;down=false;clearInterval(timer);steer=0;
  send(false);setTimeout(()=>send(false),150);   // 정지는 한 번 더(패킷 손실 대비)
  dot.style.left='50%';dot.style.background='#555';info.textContent='정지';}
pad.addEventListener('pointerup',up);pad.addEventListener('pointercancel',up);
document.addEventListener('visibilitychange',()=>{if(document.hidden)up();});
// 손 뗌 이벤트가 누락돼도 멈추게: 알림·제어 센터로 포커스를 잃거나 페이지를 떠나면 정지
window.addEventListener('blur',up);window.addEventListener('pagehide',up);
document.addEventListener('touchend',e=>{if(e.touches.length===0)up();});
document.addEventListener('touchcancel',up);

// ---- Pi 연결 상태 (0.5초마다) ----
setInterval(()=>{
  const ctl=new AbortController();const t=setTimeout(()=>ctl.abort(),800);
  fetch('/state',{cache:'no-store',signal:ctl.signal}).then(r=>{
    clearTimeout(t);link.className='';link.textContent='Pi 연결됨';
  }).catch(()=>{link.className='bad';link.textContent='⚠ Pi 연결 끊김 — 차가 곧 스스로 정지';});
},500);
</script></body></html>"""


def command(go, steer, age, speed, max_steer, timeout):
    """휴대폰 입력 → (linear_x, angular_z). 손 뗌·신호 끊김이면 정지.

    steer는 화면 위치 -1(왼쪽 끝)~+1(오른쪽 끝). REP-103에서 angular.z +는 좌회전이므로
    오른쪽으로 밀면 음수가 되도록 부호를 뒤집는다.
    """
    if not go or age > timeout:
        return 0.0, 0.0
    steer = max(-1.0, min(1.0, steer))
    return speed, -steer * max_steer


def accept(last, sid, seq):
    """요청을 받아들일지. last = (sid, seq) 마지막으로 받아들인 것, 없으면 None.

    HTTP 요청은 도착 순서가 뒤바뀔 수 있다. '정지' 뒤에 늦게 도착한 옛 '전진'을 받아들이면
    차가 다시 움직이므로, 같은 탭에서 이미 처리한 순번 이하는 버린다.
    다른 탭(sid 다름, 예: 새로고침)이면 새로 시작한다.
    """
    return last is None or sid != last[0] or seq > last[1]


def _self_check():
    # 순서 뒤바뀐 요청 거르기 — 정지(seq 10) 뒤에 옛 전진(seq 9)이 오면 버린다
    assert accept(None, 'a', 1)
    assert accept(('a', 9), 'a', 10)
    assert not accept(('a', 10), 'a', 9)
    assert not accept(('a', 10), 'a', 10)
    assert accept(('a', 500), 'b', 1)          # 새로고침한 탭은 순번 1부터 다시

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

    state = {'go': False, 'steer': 0.0, 't': 0.0, 'last': None}
    lock = threading.Lock()
    page = PAGE.replace('{cam_port}', str(args.cam_port)).encode('utf-8')

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'   # 연결 유지 — 요청마다 TCP를 새로 맺지 않아 지연이 준다

        def log_message(self, *a):
            pass

        def _reply(self, code, body=b'', ctype='text/plain'):
            self.send_response(code)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            url = urlparse(self.path)
            q = parse_qs(url.query)
            if url.path == '/cmd':
                try:
                    go = q.get('go', ['0'])[0] == '1'
                    steer = float(q.get('steer', ['0'])[0])
                    sid = q.get('sid', [''])[0]
                    seq = int(q.get('seq', ['0'])[0])
                except ValueError:
                    go, steer, sid, seq = False, 0.0, '', 0
                with lock:
                    if accept(state['last'], sid, seq):
                        state.update(go=go, steer=steer, t=time.monotonic(), last=(sid, seq))
                self._reply(204)
            elif url.path == '/state':
                self._reply(204)   # 연결 확인용
            else:
                self._reply(200, page, 'text/html; charset=utf-8')

    class WebTeleop(Node):
        def __init__(self):
            super().__init__('web_teleop')
            self.pub = self.create_publisher(Twist, args.topic, 10)
            self.stop_left = 0   # 정지 후 몇 번 더 0을 보낼지
            self.moving = False
            self.create_timer(0.05, self._tick)  # 20Hz

        def _tick(self):
            with lock:
                go, steer, t = state['go'], state['steer'], state['t']

            age = time.monotonic() - t
            lin, ang = command(go, steer, age, args.speed, args.max_steer, args.timeout)
            if (lin != 0.0) != self.moving:
                self.moving = lin != 0.0
                why = '' if self.moving else (' (손 뗌)' if not go else f' (폰 신호 {age:.1f}초 끊김)')
                self.get_logger().info('전진' if self.moving else f'정지{why}')
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
        f'휴대폰에서 http://raspberrypi.local:{args.port}/ 접속 - {args.topic}, '
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
