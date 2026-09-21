#!/usr/bin/env bash
# 전부 정지. 주행 중 급히 멈춰야 할 때 이것만 실행하면 된다.
#
# 주의: 노드 이름을 변수로 쪼개서 만든다. pkill -f 패턴이 이 스크립트 본문과
#       겹치면 자기 자신을 죽여서 뒷부분이 실행되지 않는다(실제로 겪은 문제).
source /home/pi/safecar_env.sh

L="lane_follow";  M="er_node"
D="decision_mak"; N="er_node"
V="vision_det";   W="ector_node"

pkill -9 -f "$L$M"
pkill -9 -f "$D$N"
echo "[stop] 제어 노드 종료"

# 발행자가 사라지면 stella_md 워치독(0.5s)이 모터를 세우지만, 확실히 하려고 0을 직접 쏜다.
ros2 topic pub -t 5 -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" > /dev/null 2>&1
echo "[stop] 정지 명령 발행"

if [ "$1" = "all" ]; then
    pkill -9 -f "$V$W"
    pkill -9 -f camera_no""de          # SIGTERM으로 안 죽고 CSI를 물고 있어서 -9 필수
    pkill -f image_serv""er
    pkill -f stella_m""d
    pkill -9 -f ydlidar""_node        # 라이다 모터도 같이 멈춘다
    pkill -9 -f stella_ahrs""_node
    pkill -9 -f joint_state_pub""lisher
    pkill -9 -f robot_state_pub""lisher
    # 통합 런치로 띄운 경우 부모(ros2 launch)가 살아있으면 자식이 남는다
    pkill -9 -f "ros2 lau""nch"
    echo "[stop] 카메라/인지/모터드라이버까지 전부 종료"
fi

sleep 1
echo "--- /cmd_vel 발행자 수 (0이어야 정상) ---"
timeout 5 ros2 topic info /cmd_vel 2>/dev/null | grep Publisher
