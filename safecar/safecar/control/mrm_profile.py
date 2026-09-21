"""갓길 대피(MRM, Minimum Risk Manoeuvre) 주행 프로파일. (제어부, ROS 비의존 순수 로직)

운전자 이상이 확정되면 '즉시 정지'가 아니라 시간에 따라 부드럽게
① 횡방향으로 치우치며 감속 → ② 완전 정지 하는 2단 프로파일을 만든다.

UN R157(ALKS) 기준:
- MRM의 기본은 **차로 내 정지**다. 차선 변경(갓길 이동)형 MRM은 시스템이 그럴 능력이
  있을 때만 허용된다. 본 차량은 후방 감시 센서가 없으므로 갓길 이동은
  '후방 교통이 없는 폐쇄 트랙'이라는 ODD 안에서만 유효하다.
  → `lateral_bias=0.0`으로 두면 그대로 R157 기본 동작(차로 내 정지)이 된다.
- 정차 후에는 수동 입력 없이 다시 움직여선 안 된다 → 속도 0을 계속 유지한다(래치는 호출부 책임).
"""


class MrmProfile:
    """경과 시간 → (속도 배율, 횡방향 바이어스)."""

    def __init__(self, lateral_bias=0.5, transition_time=3.0,
                 speed_ratio=0.6, stop_duration=2.0):
        # 횡방향 목표 바이어스. **양수 = 우측**(오프셋 부호 규약상 우조향).
        # 0.0이면 횡이동 없이 차로 안에서 그대로 정지(R157 기본 MRM).
        self.lateral_bias = lateral_bias
        self.transition_time = max(transition_time, 1e-3)  # 갓길로 붙는 시간(초)
        self.speed_ratio = speed_ratio                      # 이동 중 유지할 속도 비율
        self.stop_duration = max(stop_duration, 1e-3)       # 붙은 뒤 정지까지 걸리는 시간(초)

    def compute(self, elapsed):
        """대피 시작 후 elapsed초 시점의 (속도 배율 0~1, 횡 바이어스)."""
        if elapsed < 0.0:
            return 1.0, 0.0

        if elapsed < self.transition_time:
            # ① 이동 구간: 바이어스를 0→목표로 올리며 속도를 cruise→cruise*ratio로 낮춘다.
            k = elapsed / self.transition_time
            return 1.0 - (1.0 - self.speed_ratio) * k, self.lateral_bias * k

        # ② 정지 구간: 바이어스는 목표값 유지, 속도만 0으로 내린다.
        k = (elapsed - self.transition_time) / self.stop_duration
        if k >= 1.0:
            return 0.0, self.lateral_bias
        return self.speed_ratio * (1.0 - k), self.lateral_bias


def _self_check():
    p = MrmProfile(lateral_bias=0.5, transition_time=3.0, speed_ratio=0.6, stop_duration=2.0)

    # 시작 순간: 평소 속도, 아직 안 치우침
    assert p.compute(0.0) == (1.0, 0.0)

    # 이동 구간 중간: 속도는 1.0~0.6 사이, 바이어스는 0~0.5 사이로 단조 증가
    s1, b1 = p.compute(1.5)
    assert 0.6 < s1 < 1.0 and abs(b1 - 0.25) < 1e-9
    s2, b2 = p.compute(2.9)
    assert s2 < s1 and b2 > b1

    # 이동 완료 직후: 목표 바이어스 도달, 속도는 ratio
    s3, b3 = p.compute(3.0)
    assert abs(s3 - 0.6) < 1e-9 and abs(b3 - 0.5) < 1e-9

    # 정지 구간: 속도만 줄고 바이어스는 유지
    s4, b4 = p.compute(4.0)
    assert abs(s4 - 0.3) < 1e-9 and abs(b4 - 0.5) < 1e-9

    # 완전 정지 후에는 계속 0 (재출발 금지 — R157)
    assert p.compute(5.0) == (0.0, 0.5)
    assert p.compute(60.0) == (0.0, 0.5)

    # lateral_bias=0 이면 횡이동 없는 차로 내 정지
    q = MrmProfile(lateral_bias=0.0)
    assert all(q.compute(t)[1] == 0.0 for t in (0.0, 1.0, 3.0, 10.0))
    assert q.compute(10.0)[0] == 0.0

    print("MrmProfile self-check OK")


if __name__ == '__main__':
    _self_check()
