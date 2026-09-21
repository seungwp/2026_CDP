#!/usr/bin/env bash
# 주행 준비: 통합 launch(safecar.launch.py)로 차체·센서·인지·안전게이트를 전부 띄우고,
# 노트북 브라우저로 볼 수 있게 영상 스트리머를 붙인다. **주행은 시작하지 않는다**
# (차선 추종은 safecar_drive.sh로 따로 켠다).
#
# 실행 경로는 launch 하나다. 예전에는 이 스크립트가 노드를 하나씩 띄웠는데, 그러다
# launch와 켜지는 노드가 달라져(Hailo·IMU·sensor_bridge 누락) 스크립트로 달리면
# 장애물 정지가 없는 채로 주행했다.
#
#   ~/safecar_start.sh                    # 기본
#   ~/safecar_start.sh anomaly_delay_sec:=10.0   # 웹캠 없이 MRM 데모(10초 뒤 이상 발생)
source /home/pi/safecar_env.sh
mkdir -p /home/pi/runlog && cd /home/pi/runlog

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
echo
echo "영상:  http://raspberrypi.local:8080/   (차선 검출 결과)"
echo "       http://raspberrypi.local:8081/   (원본)"
echo "주행 시작:  ~/safecar_drive.sh"
echo "정지:       ~/safecar_stop.sh        (주행만)  /  ~/safecar_stop.sh all  (전부)"
