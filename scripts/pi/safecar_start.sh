#!/usr/bin/env bash
# 주행 준비: 통합 launch(safecar.launch.py)로 차체·센서·인지·안전게이트를 전부 띄우고,
# 노트북 브라우저로 볼 수 있게 영상 스트리머를 붙인다. **주행은 시작하지 않는다**
# (차선 추종은 safecar_drive.sh로 따로 켠다).
#
# 실행 경로는 launch 하나다. 예전에는 이 스크립트가 노드를 하나씩 띄웠는데, 그러다
# launch와 켜지는 노드가 달라져(Hailo·IMU·sensor_bridge 누락) 스크립트로 달리면
# 장애물 정지가 없는 채로 주행했다.
#
#   ~/safecar_start.sh                    # 운전자 신호 = 노트북 웹캠(drowsy_v5.py, UDP 5005)
#   ~/safecar_start.sh bio_source:=sim anomaly_delay_sec:=10.0   # 웹캠 없이 MRM 데모(10초 뒤 이상 발생)
source /home/pi/safecar_env.sh
mkdir -p /home/pi/runlog && cd /home/pi/runlog

# 이전 실행이 강제 종료되면 /dev/shm에 FastDDS 찌꺼기가 남고, 쌓이면 새 노드가
# 포트를 못 잡는다("Failed init_port fastrtps_port7000"). 남은 노드가 하나도 없을
# 때만 지운다 — 돌고 있는데 지우면 그 노드들 통신이 끊긴다.
if ! pgrep -f "safecar/lib|camera_node|ydlidar_node|Hailo Detection" > /dev/null; then
    ros2 daemon stop > /dev/null 2>&1      # 데몬도 shm을 쓴다. 다음 ros2 명령에 자동 재시작
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
fi

start() {  # start <로그이름> <명령...>
    local log="$1"; shift
    setsid nohup "$@" > "$log" 2>&1 < /dev/null &
}

start launch.log ros2 launch safecar safecar.launch.py "$@"
start img.log    python3 /home/pi/image_server.py --topic /perception/lane_image
start imgraw.log python3 /home/pi/image_server.py --topic /camera/image_raw --port 8081
sleep 20

echo "=== 실행 상태 ==="
for t in /scan /camera/image_raw /perception/lane_offset /perception/obstacle_detected /control/driving_state; do
    printf "%-32s " "$t"
    timeout 6 ros2 topic hz "$t" 2>&1 | grep -m1 "average rate" || echo "(발행 없음)"
done

# 같은 노드가 둘 이상 뜨면 /cmd_vel이나 USB 시리얼을 두고 싸운다(실측: v2x 8개).
# image_server는 8080(검출)·8081(원본) 두 개를 일부러 띄우므로 제외한다.
DUP=$(ros2 node list 2>/dev/null | grep -v '^/image_server$' | sort | uniq -d)
if [ -n "$DUP" ]; then
    echo "!! 중복 노드 — ~/safecar_stop.sh all 로 정리 후 다시 실행할 것:"
    echo "$DUP"
fi
echo
# 핫스팟에서는 mDNS(raspberrypi.local)가 자주 안 먹는다 — 실제 IP도 같이 찍는다.
IP=$(hostname -I | awk '{print $1}')
echo "영상:  http://${IP:-raspberrypi.local}:8080/   (차선 검출 결과)"
echo "       http://${IP:-raspberrypi.local}:8081/   (원본)"
echo "노트북 졸음감지:  py -3.9 drowsy_v5.py --pi ${IP:-raspberrypi.local}"
echo "주행 시작:  ~/safecar_drive.sh"
echo "정지:       ~/safecar_stop.sh        (주행만)  /  ~/safecar_stop.sh all  (전부)"
