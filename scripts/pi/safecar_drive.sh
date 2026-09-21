#!/usr/bin/env bash
# 차선 추종 주행 시작. Ctrl+C 로 멈춘다.
#
#   ~/safecar_drive.sh                  # 확정된 튜닝값으로 주행
#   ~/safecar_drive.sh 0.15 0.7         # 속도, 조향게인(횡오차)
#   ~/safecar_drive.sh 0.15 0.55 0.4    # + 헤딩게인(곡선 선제 조향)
#
# 2026-09-21 실외 트랙 실측으로 정한 값:
#   속도 0.12 미만은 정지 마찰을 못 이겨 아예 안 움직인다.
#   조향게인 1.2는 발산 진동(회전반경 8cm), 0.4~0.55에서 안정.
source /home/pi/safecar_env.sh
SPEED="${1:-0.15}"
GAIN="${2:-0.55}"
HEAD="${3:-0.25}"   # 헤딩게인: 곡선에서 밀리면 올리고, 곡선에서 급하면 내린다
echo "주행 시작 — 속도 $SPEED m/s, 조향게인 $GAIN, 헤딩게인 $HEAD   (Ctrl+C 로 정지)"
echo "멈춘 뒤 차가 계속 가면 다른 터미널에서: ~/safecar_stop.sh"
ros2 run safecar_control lane_follower_node --ros-args \
    -p cruise_speed:="$SPEED" -p steer_gain:="$GAIN" -p steer_head_gain:="$HEAD"
