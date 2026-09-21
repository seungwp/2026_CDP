#!/usr/bin/env python3
"""모방학습 주행 모델 학습 (노트북에서 실행, ROS 불필요).

record_dataset.py로 Pi에서 녹화한 세션 폴더들을 받아 "영상 → 조향(angular.z)" CNN을 학습하고,
Pi의 bc_follower_node가 OpenCV DNN으로 읽을 수 있는 ONNX로 내보낸다.

준비:
    pip install torch onnx numpy opencv-python     # GPU가 있으면 CUDA 빌드 torch
    # Pi에서 데이터 가져오기 (노트북 PowerShell)
    scp -r pi@raspberrypi.local:~/dataset .\\dataset

학습:
    python scripts/train_bc.py dataset/*               # 세션 폴더 여러 개
    python scripts/train_bc.py dataset/* --epochs 30 --out bc_model.onnx

    # Pi로 모델 보내기
    scp bc_model.onnx pi@raspberrypi.local:~/

- 검증 데이터는 세션마다 **마지막 10%**를 쓴다. 무작위로 뽑으면 바로 옆 프레임(거의 같은 사진)이
  학습/검증에 나뉘어 들어가 점수가 부풀려진다.
- 좌우 반전(조향 부호 반전)과 밝기 변화로 데이터를 늘린다. 반전은 곡선 방향 편향을 없애준다.
- 끝나면 ONNX를 OpenCV DNN으로 다시 읽어 PyTorch와 출력이 같은지 확인한다(Pi에서 못 읽는 모델 방지).
"""

import argparse
import csv
import glob
import os
import sys

import cv2
import numpy as np
import torch
import torch.nn as nn

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'safecar'))
from safecar.control.bc_model import IN_H, IN_W, preprocess  # noqa: E402


class PilotNet(nn.Module):
    """NVIDIA PilotNet(2016)을 입력 160x64에 맞춰 줄인 것. Pi CPU에서 수 ms."""

    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(3, 24, 5, stride=2), nn.ReLU(),
            nn.Conv2d(24, 36, 5, stride=2), nn.ReLU(),
            nn.Conv2d(36, 48, 5, stride=2), nn.ReLU(),
            nn.Conv2d(48, 64, 3), nn.ReLU(),
            nn.Flatten(),
            nn.Linear(64 * 3 * 15, 100), nn.ReLU(), nn.Dropout(0.3),
            nn.Linear(100, 50), nn.ReLU(),
            nn.Linear(50, 1),
        )

    def forward(self, x):
        return self.net(x)


def load_sessions(dirs, min_speed):
    """→ (train_x, train_y, val_x, val_y). 이미지는 메모리를 아끼려고 uint8로 들고 있는다."""
    tx, ty, vx, vy = [], [], [], []
    for d in dirs:
        path = os.path.join(d, 'labels.csv')
        if not os.path.exists(path):
            print(f'  건너뜀(labels.csv 없음): {d}')
            continue
        with open(path, newline='') as f:
            rows = [r for r in csv.DictReader(f) if abs(float(r['linear_x'])) >= min_speed]
        xs, ys = [], []
        for r in rows:
            img = cv2.imread(os.path.join(d, 'images', r['frame']))
            if img is None:
                continue
            x = preprocess(img)
            xs.append(np.round((x + 0.5) * 255.0).astype(np.uint8))  # preprocess의 역변환(손실 없음)
            ys.append(float(r['angular_z']))
        n_val = max(1, len(xs) // 10)
        tx += xs[:-n_val]; ty += ys[:-n_val]
        vx += xs[-n_val:]; vy += ys[-n_val:]
        print(f'  {os.path.basename(os.path.normpath(d))}: {len(xs)}장')
    to = lambda a: np.stack(a) if a else np.zeros((0, 3, IN_H, IN_W), np.uint8)
    return to(tx), np.array(ty, np.float32), to(vx), np.array(vy, np.float32)


def to_input(u8):
    """uint8 텐서 → 모델 입력. preprocess()와 같은 식이어야 한다."""
    return u8.float() / 255.0 - 0.5


def augment(u8, y):
    x = to_input(u8)
    flip = torch.rand(len(x), device=x.device) < 0.5
    x[flip] = x[flip].flip(-1)                       # 좌우 반전
    y = torch.where(flip, -y, y)                     # → 조향도 반대로
    gain = torch.empty(len(x), 1, 1, 1, device=x.device).uniform_(0.7, 1.3)
    x = ((x + 0.5) * gain).clamp(0, 1) - 0.5         # 밝기(햇빛·그늘)
    return x, y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('sessions', nargs='+', help='record_dataset.py 세션 폴더들 (glob 가능)')
    ap.add_argument('--epochs', type=int, default=20)
    ap.add_argument('--batch', type=int, default=64)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--min-speed', type=float, default=0.01)
    ap.add_argument('--out', default='bc_model.onnx')
    args = ap.parse_args()

    dirs = sorted({p for s in args.sessions for p in glob.glob(s) if os.path.isdir(p)})
    print(f'세션 {len(dirs)}개 읽는 중')
    tx, ty, vx, vy = load_sessions(dirs, args.min_speed)
    if len(tx) < 100:
        raise SystemExit(f'학습 데이터가 너무 적다({len(tx)}장). 녹화를 더 하세요.')

    uniq = len(np.unique(np.round(ty, 3)))
    print(f'학습 {len(tx)}장 / 검증 {len(vx)}장, 조향 범위 {ty.min():+.2f}~{ty.max():+.2f}, 서로 다른 조향값 {uniq}개')
    if uniq < 10:
        print('[경고] 조향값 종류가 너무 적다 - 키보드처럼 계단식 조종이면 모델이 끊기듯 조향한다. 아날로그 스틱 권장.')

    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    model = PilotNet().to(dev)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)
    tx_t, ty_t = torch.from_numpy(tx).to(dev), torch.from_numpy(ty).to(dev)
    vx_t, vy_t = torch.from_numpy(vx).to(dev), torch.from_numpy(vy).to(dev)
    baseline = float(np.abs(vy - ty.mean()).mean())  # 항상 평균값만 내는 모델의 오차
    print(f'기준선(평균만 예측) 검증 MAE {baseline:.4f} - 이보다 충분히 낮아야 뭔가 배운 것\n')

    best, best_state = float('inf'), None
    for ep in range(1, args.epochs + 1):
        model.train()
        perm = torch.randperm(len(tx_t), device=dev)
        total = 0.0
        for i in range(0, len(perm), args.batch):
            idx = perm[i:i + args.batch]
            x, y = augment(tx_t[idx], ty_t[idx])
            loss = nn.functional.mse_loss(model(x).squeeze(1), y)
            opt.zero_grad()
            loss.backward()
            opt.step()
            total += loss.item() * len(idx)
        model.eval()
        with torch.no_grad():
            pred = torch.cat([model(to_input(vx_t[i:i + 256])).squeeze(1)
                              for i in range(0, len(vx_t), 256)])
            mae = (pred - vy_t).abs().mean().item()
        mark = ''
        if mae < best:
            best, best_state, mark = mae, {k: v.clone() for k, v in model.state_dict().items()}, ' *'
        print(f'epoch {ep:3d}  train MSE {total / len(tx_t):.4f}  val MAE {mae:.4f}{mark}')

    model.load_state_dict(best_state)
    model.eval().cpu()
    dummy = torch.zeros(1, 3, IN_H, IN_W)
    torch.onnx.export(model, dummy, args.out, input_names=['image'], output_names=['steer'],
                      opset_version=11, dynamo=False)

    # Pi와 같은 방식(OpenCV DNN)으로 다시 읽어 출력이 같은지 확인
    net = cv2.dnn.readNetFromONNX(args.out)
    sample = to_input(torch.from_numpy(vx[:32]))
    net.setInput(sample.numpy())
    cv_out = net.forward().reshape(-1)
    with torch.no_grad():
        pt_out = model(sample).squeeze(1).numpy()
    diff = float(np.abs(cv_out - pt_out).max())
    assert diff < 1e-3, f'OpenCV DNN 출력이 PyTorch와 다르다(최대 차이 {diff})'
    print(f'\n저장: {args.out}  (best val MAE {best:.4f}, 기준선 {baseline:.4f}, OpenCV 검증 차이 {diff:.1e})')


if __name__ == '__main__':
    main()
