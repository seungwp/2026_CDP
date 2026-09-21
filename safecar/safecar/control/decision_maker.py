from safecar.protocol import (
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


def judge_obstacle(hailo_detected, camera_fresh, hailo_fresh, scan_fresh, front_min,
                   confirm_m, emergency_m, require_perception=True, fuse_lidar=True):
    """카메라(Hailo)와 라이다를 합쳐 '지금 멈춰야 하는가'를 판단한다. → (정지 여부, 이유)

    카메라 한 대의 한계를 라이다로 보완한다:
    - Hailo는 **무엇인지**는 알지만 **거리를 모른다** → 10m 밖 사람에도 섰다.
    - Hailo는 학습한 종류(사람·차 등)만 본다 → 박스·콘 앞에서는 안 섰다.
    - 라이다는 **거리를 직접 재지만** 종류를 모른다.

    판단 순서 (위가 우선):
    1) 인지가 살아있는가 — 카메라나 Hailo가 끊기면 정지(fail-safe).
       Hailo 노드는 마지막 판정을 타이머로 계속 재발행하므로, 카메라가 죽어도
       'False'가 계속 나온다. 그래서 카메라 자체의 수신 여부를 따로 본다.
    2) 라이다 전방이 emergency_m보다 가까우면 **종류와 무관하게** 정지 (박스 등).
    3) Hailo가 감지했고 라이다 전방 confirm_m 안에 실제로 뭔가 있으면 정지.
    4) Hailo가 감지했어도 전방 confirm_m 안이 비어 있으면 **먼 물체로 보고 무시**.
    라이다가 없으면(끊김/꺼둠) 예전처럼 Hailo 단독으로 판단한다.

    front_min: 라이다 전방 섹터 최소거리(m). None이면 반사 없음 = 비어 있음.

    한계: 라이다는 자기 높이의 수평면만 본다. 그 평면보다 낮은 물체(작은 아이 등)는
    Hailo가 봐도 라이다가 못 보므로 4)에 걸려 무시될 수 있다. 시연 장애물은 라이다
    높이보다 높은 것을 쓰고, 필요하면 fuse_lidar=False로 Hailo 단독 판단으로 되돌린다.
    """
    if require_perception and not camera_fresh:
        return True, '카메라 영상 끊김 (fail-safe)'
    if require_perception and not hailo_fresh:
        return True, 'Hailo 응답 없음 (fail-safe)'

    if fuse_lidar and scan_fresh:
        if front_min is not None and front_min < emergency_m:
            return True, f'라이다 전방 {front_min:.2f}m 근접 (종류 무관 정지)'
        if hailo_detected:
            if front_min is not None and front_min < confirm_m:
                return True, f'Hailo 감지 + 라이다 전방 {front_min:.2f}m 확인'
            return False, f'Hailo 감지했으나 전방 {confirm_m:.1f}m 안이 비어 있음 (먼 물체, 무시)'
        return False, ''

    if hailo_detected:
        return True, 'Hailo 감지 (라이다 없음, 단독 판단)'
    return False, ''


def _self_check():
    base = dict(confirm_m=1.0, emergency_m=0.3)
    ok = dict(camera_fresh=True, hailo_fresh=True, scan_fresh=True)

    # B: 인지가 끊기면 무조건 정지
    assert judge_obstacle(False, False, True, True, None, **base)[0] is True
    assert judge_obstacle(False, True, False, True, None, **base)[0] is True
    # 인지 감시를 끄면(벤치 테스트) 끊겨도 막지 않는다
    assert judge_obstacle(False, False, False, True, None, require_perception=False, **base)[0] is False

    # A: 라이다 근접 — Hailo가 못 봐도(박스 등) 정지
    assert judge_obstacle(False, front_min=0.2, **ok, **base)[0] is True
    # 사람 + 가까움 → 정지
    assert judge_obstacle(True, front_min=0.6, **ok, **base)[0] is True
    # 사람인데 멀다(전방 1m 안 비어 있음) → 무시 = 먼 사람에 급정거하던 문제 해결
    assert judge_obstacle(True, front_min=3.0, **ok, **base)[0] is False
    assert judge_obstacle(True, front_min=None, **ok, **base)[0] is False
    # 아무것도 없음 → 주행
    assert judge_obstacle(False, front_min=None, **ok, **base) == (False, '')

    # 라이다가 끊기면 Hailo 단독 (감지를 버리지 않는다)
    assert judge_obstacle(True, True, True, False, None, **base)[0] is True
    assert judge_obstacle(False, True, True, False, None, **base)[0] is False
    # 퓨전을 끄면 Hailo 단독
    assert judge_obstacle(True, front_min=3.0, fuse_lidar=False, **ok, **base)[0] is True

    # 우선순위: 장애물 > 운전자 이상 > 정상
    d = DecisionMaker()
    assert d.decide(True, True) == COMMAND_EMERGENCY_BRAKE
    assert d.decide(True, False) == COMMAND_MRM_PULL_OVER
    assert d.decide(False, False) == COMMAND_NORMAL
    print('decision_maker self-check OK')


if __name__ == '__main__':
    # 실행: cd safecar && python3 -m safecar.control.decision_maker
    _self_check()
