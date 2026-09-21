#!/usr/bin/env bash
# 정지.
#   ~/safecar_stop.sh        주행만 멈춘다(차선 추종 종료 + 정지 명령). 센서·게이트는 살려둔다.
#   ~/safecar_stop.sh all    launch째로 전부 끈다(카메라·라이다 모터까지).
#
# 주의: pkill 패턴을 변수로 쪼개서 만든다. 패턴이 이 스크립트를 실행하는 셸의
#       명령줄과 겹치면 자기 자신을 죽여 뒷부분이 실행되지 않는다(실제로 겪은 문제).
#       그리고 `ros2 run`의 PID만 죽이면 실제 노드가 살아남는다 — 이름으로 죽인다.
source /home/pi/safecar_env.sh

LF="lane_follow""er_node"
pkill -9 -f "$LF"
# 게이트는 /cmd_vel_raw가 1초 끊기면 스스로 0을 내지만, 기다리지 않고 바로 0을 쏜다.
ros2 topic pub -t 5 -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" > /dev/null 2>&1
echo "[stop] 주행 정지"

if [ "$1" = "all" ]; then
    pkill -9 -f "ros2 lau""nch"      # 부모가 살아있으면 자식이 남는다
    for n in decision_mak""er_node vision_det""ector_node sensor_brid""ge_node \
             hailo_ros2_detec""tion_node stella_m""d_node stella_ah""rs_node \
             ydlidar""_node camera_no""de image_serv""er; do
        pkill -9 -f "$n"             # camera_node는 SIGTERM으로 안 죽고 CSI를 물고 있다
    done
    echo "[stop] launch 전체 종료 (카메라·라이다 포함)"
fi

sleep 1
echo "--- /cmd_vel 발행자 수 (all이면 0이어야 정상) ---"
timeout 5 ros2 topic info /cmd_vel 2>/dev/null | grep Publisher || echo "Publisher count: 0"
