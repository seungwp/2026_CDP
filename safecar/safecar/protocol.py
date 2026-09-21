"""
인지·제어·통신 모듈이 공통으로 사용하는 명령어·상태 값 정의.
새 명령어나 상태를 추가/변경할 때는 이 파일만 고치면 된다.
'/control/driving_state' 토픽(std_msgs/String)의 data 값으로 그대로 실린다.
"""

# 제어부(safecar.control.decision_maker)가 판단해서 내리는 주행 상태
COMMAND_NORMAL = "NORMAL"                    # 정상 주행
COMMAND_EMERGENCY_BRAKE = "EMERGENCY_BRAKE"  # 현재 차선 급제동 (전방 장애물 있음)
COMMAND_MRM_PULL_OVER = "MRM_PULL_OVER"      # 갓길 대피 (전방 공간 확보됨)
