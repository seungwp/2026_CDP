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
        # 1. 전방 장애물: 운전자 상태와 무관하게 최우선 정지 (자율주행 중 AEB)
        if obstacle_detected:
            return COMMAND_EMERGENCY_BRAKE

        # 2. 운전자 이상 (전방은 비어 있음): 우측 갓길 대피
        if bio_anomaly:
            return COMMAND_MRM_PULL_OVER

        # 3. 정상 주행
        return COMMAND_NORMAL
