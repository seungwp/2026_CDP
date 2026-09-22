#!/usr/bin/env bash
# 차선 추종 주행 시작. Ctrl+C 로 멈춘다.
#
#   ~/safecar_drive.sh                  # 확정된 튜닝값으로 주행
#   ~/safecar_drive.sh 0.15 0.7         # 속도, 조향게인(횡오차)
#   ~/safecar_drive.sh 0.15 0.55 0.4    # + 헤딩게인(곡선 선제 조향)
#
# 인자를 안 주면 노드 기본값(= 2026-09-21 실외 트랙 실측값)으로 달린다:
#   속도 0.15 / 조향게인 0.55 / 헤딩게인 0.25
#
# 모방학습 모델로 주행 (~/bc_model.onnx 필요):
#   ~/safecar_drive.sh bc               # 속도 0.3, 조향배율 1.0
#   ~/safecar_drive.sh bc 0.3 0.8       # 속도, 조향배율(흔들리면 ↓, 곡선에서 밀리면 ↑)
#
# bc 기본 속도가 0.3인 이유: 모델은 조향을 **각속도(rad/s)** 로 배우는데, 각속도 =
# 속도 × 곡률이라 녹화할 때와 다른 속도로 달리면 같은 조향값이 다른 궤적을 그린다.
# 학습 데이터(dataset/20260922_131811, 30,379장)가 전부 0.3 m/s에서 녹화됐다.
# (실측 확인: 같은 트랙을 0.15로 녹화한 옛 데이터는 |조향| 중앙값 0.036,
#  0.3으로 녹화한 데이터는 0.066으로 정확히 2배였다.)
# 다른 속도로 달리려면 조향배율을 속도비로 주면 된다 — 예: 0.15면 배율 0.5.
#
# 먼저 ~/safecar_start.sh 로 센서·게이트를 띄워둬야 한다.
source /home/pi/safecar_env.sh
if [ "$1" = "bc" ]; then
    SPEED="${2:-0.3}"
    SCALE="${3:-1.0}"
    echo "모방학습 주행 + MRM 갓길정차(노란선 인식) — 속도 $SPEED m/s, 조향배율 $SCALE   (Ctrl+C 로 정지)"
    echo "멈춘 뒤 차가 계속 가면 다른 터미널에서: ~/safecar_stop.sh"
    # 평소엔 bc_follower가 몰고, 운전자 이상 신호가 오면 lane_follower가 조용히 이어받아
    # 노란 갓길선으로 붙는다(drive_normal:=false라 평소엔 아무것도 안 냄) — 둘 다 같은
    # driving_state를 보고 한쪽만 활성화되므로 /cmd_vel_raw를 두고 싸우지 않는다.
    # cruise_speed를 BC와 맞춰준다 — 안 맞추면 MRM 전환 순간 lane_follower의 기본값(0.15)으로
    # 속도가 튄다(감속 곡선이 시작되기도 전에 훅 느려지는 것처럼 보임).
    # 조향 게인도 같이 키운다 — 게인 0.55/0.25는 0.15 m/s에서 맞춘 값이라, 속도만 올리면
    # 같은 조향 각속도로 곡률이 절반이 되어 갓길로 덜 붙는다(BC 조향배율과 같은 이유).
    LF_GAIN=$(awk -v s="$SPEED" 'BEGIN{printf "%.3f", 0.55*s/0.15}')
    LF_HEAD=$(awk -v s="$SPEED" 'BEGIN{printf "%.3f", 0.25*s/0.15}')
    echo "MRM 조향게인 $LF_GAIN / 헤딩게인 $LF_HEAD (0.15 m/s 기준값 × 속도비)"
    ros2 run safecar lane_follower_node --ros-args -p drive_normal:=false \
        -p cruise_speed:="$SPEED" -p steer_gain:="$LF_GAIN" -p steer_head_gain:="$LF_HEAD" &
    LF_PID=$!
    trap 'kill -9 "$LF_PID" 2>/dev/null' EXIT
    ros2 run safecar bc_follower_node --ros-args \
        -p cruise_speed:="$SPEED" -p steer_scale:="$SCALE"
    exit
fi
SPEED="${1:-0.15}"   # 0.12 미만이면 정지 마찰 때문에 안 움직인다
GAIN="${2:-0.55}"
HEAD="${3:-0.25}"   # 헤딩게인: 곡선에서 밀리면 올리고, 곡선에서 급하면 내린다
echo "주행 시작 — 속도 $SPEED m/s, 조향게인 $GAIN, 헤딩게인 $HEAD   (Ctrl+C 로 정지)"
echo "멈춘 뒤 차가 계속 가면 다른 터미널에서: ~/safecar_stop.sh"
ros2 run safecar lane_follower_node --ros-args \
    -p cruise_speed:="$SPEED" -p steer_gain:="$GAIN" -p steer_head_gain:="$HEAD"
