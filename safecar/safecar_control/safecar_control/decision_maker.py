from safecar_msgs.command_protocol import (
    COMMAND_NORMAL,
    COMMAND_EMERGENCY_BRAKE,
    COMMAND_MRM_PULL_OVER,
)


class DecisionMaker:
    """운전자 상태 + 전방 장애물 여부를 입력받아 주행 명령을 결정한다. (제어부, ROS 비의존 순수 로직)"""

    def __init__(self):
        print("[System] Decision Maker 초기화 완료.")

    def decide(self, bio_anomaly, obstacle_detected):
        # 1. 운전자 상태 정상
        if not bio_anomaly:
            return COMMAND_NORMAL

        # 2. 운전자 이상 발생 시: 전방 상황에 따른 페일세이프(MRM) 이중 개입
        if obstacle_detected:
            return COMMAND_EMERGENCY_BRAKE  # 현 차선 급제동 (장애물 있음)
        else:
            return COMMAND_MRM_PULL_OVER    # 우측 갓길 대피 (공간 확보됨)
