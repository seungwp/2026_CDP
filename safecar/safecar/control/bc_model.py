"""모방학습(behavior cloning) 주행 모델의 전처리. (제어부, ROS 비의존 순수 로직)

학습(노트북, scripts/train_bc.py)과 주행(Pi, bc_follower_node)이 **이 함수 하나를 같이 쓴다.**
모방학습에서 제일 흔한 버그가 학습 때와 주행 때 전처리가 달라지는 것이라, 절대 따로 구현하지 말 것.

입력: BGR 이미지 (카메라 원본 640x480 또는 record_dataset.py가 저장한 320x240 어느 쪽이든)
출력: float32 (3, IN_H, IN_W), 값 범위 -0.5~0.5
"""

import cv2
import numpy as np

REC_W, REC_H = 320, 240   # record_dataset.py 저장 크기. 카메라 원본도 먼저 여기로 맞춘다
CROP_TOP = 0.35           # 위쪽(하늘·배경)은 잘라낸다. vision_detector의 ROI_TOP과 같은 근거
IN_W, IN_H = 160, 64      # 모델 입력 크기


def preprocess(bgr):
    if bgr.shape[1] != REC_W or bgr.shape[0] != REC_H:
        bgr = cv2.resize(bgr, (REC_W, REC_H), interpolation=cv2.INTER_AREA)
    roi = bgr[int(REC_H * CROP_TOP):, :]
    roi = cv2.resize(roi, (IN_W, IN_H), interpolation=cv2.INTER_AREA)
    x = roi.astype(np.float32) / 255.0 - 0.5
    return np.ascontiguousarray(x.transpose(2, 0, 1))


def _self_check():
    raw = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    rec = cv2.resize(raw, (REC_W, REC_H), interpolation=cv2.INTER_AREA)

    x = preprocess(raw)
    assert x.shape == (3, IN_H, IN_W) and x.dtype == np.float32
    assert -0.5 <= x.min() and x.max() <= 0.5

    # 카메라 원본(640x480)과 녹화본(320x240)이 같은 입력이 되어야 한다 — 학습/주행 일치의 핵심.
    # (녹화본은 JPEG 손실이 추가로 있지만 크기 변환 경로는 동일해야 한다)
    assert np.array_equal(preprocess(raw), preprocess(rec))

    # 위쪽을 바꿔도 결과가 같다 = 잘라낸 영역은 모델에 안 들어간다
    top_changed = rec.copy()
    top_changed[:int(REC_H * CROP_TOP) - 1] = 0
    assert np.array_equal(preprocess(rec), preprocess(top_changed))
    print('bc_model self-check OK')


if __name__ == '__main__':
    # 실행: cd safecar && python3 -m safecar.control.bc_model
    _self_check()
