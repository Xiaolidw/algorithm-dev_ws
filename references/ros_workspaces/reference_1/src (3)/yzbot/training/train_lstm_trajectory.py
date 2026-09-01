#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
train_lstm_trajectory.py

使用合成数据训练 LSTMTrajectoryModel：
- 轨迹长度：obs_len + pred_len = 8 + 12 = 20
- 轨迹类型（6类）：
  1. 直线匀速
  2. 直线加减速
  3. 圆弧
  4. S 型正弦曲线
  5. Zigzag 之字形
  6. 8 字型（figure-eight）

数据规模：
- 训练集：每类 1500 条 → 共 9000 条
- 验证集：每类 300 条 → 共 1800 条

训练设置：
- epoch = 1000
- 每轮都跑完整个 train / val
- 仅当 val_loss 变好时保存 best model 到 trajectory_lstm.pth
"""

import math
import os
import random
from typing import Tuple, List

import torch
from torch import nn
from torch.utils.data import Dataset, DataLoader

from lstm_trajectory_model import LSTMTrajectoryModel


# ==================== 一些通用工具 ====================

def set_seed(seed: int = 42):
    random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)


# ==================== 合成轨迹生成函数 ====================

def generate_straight_constant(total_len: int) -> torch.Tensor:
    """直线匀速"""
    x0 = random.uniform(-5.0, 5.0)
    y0 = random.uniform(-5.0, 5.0)
    theta = random.uniform(-math.pi, math.pi)
    v = random.uniform(0.2, 1.0)  # 速度

    pts = []
    for t in range(total_len):
        dt = t * 0.1
        x = x0 + v * math.cos(theta) * dt
        y = y0 + v * math.sin(theta) * dt
        x += random.gauss(0.0, 0.01)
        y += random.gauss(0.0, 0.01)
        pts.append([x, y])
    return torch.tensor(pts, dtype=torch.float32)


def generate_straight_accel(total_len: int) -> torch.Tensor:
    """直线加减速"""
    x0 = random.uniform(-5.0, 5.0)
    y0 = random.uniform(-5.0, 5.0)
    theta = random.uniform(-math.pi, math.pi)
    v0 = random.uniform(0.1, 0.3)
    a = random.uniform(-0.02, 0.05)  # 有可能减速也可能加速

    pts = []
    for t in range(total_len):
        dt = t * 0.1
        v_t = max(0.05, v0 + a * dt)
        x = x0 + v_t * math.cos(theta) * dt
        y = y0 + v_t * math.sin(theta) * dt
        x += random.gauss(0.0, 0.01)
        y += random.gauss(0.0, 0.01)
        pts.append([x, y])
    return torch.tensor(pts, dtype=torch.float32)


def generate_circle_arc(total_len: int) -> torch.Tensor:
    """圆弧轨迹"""
    cx = random.uniform(-3.0, 3.0)
    cy = random.uniform(-3.0, 3.0)
    r = random.uniform(1.0, 3.0)
    omega = random.uniform(0.2, 1.0)  # 角速度
    theta0 = random.uniform(-math.pi, math.pi)

    pts = []
    for t in range(total_len):
        dt = t * 0.1
        theta = theta0 + omega * dt
        x = cx + r * math.cos(theta)
        y = cy + r * math.sin(theta)
        x += random.gauss(0.0, 0.01)
        y += random.gauss(0.0, 0.01)
        pts.append([x, y])
    return torch.tensor(pts, dtype=torch.float32)


def generate_s_curve(total_len: int) -> torch.Tensor:
    """S 型 / 正弦轨迹：x 匀速，y = A sin(kx)"""
    x0 = random.uniform(-4.0, -2.0)
    y0 = random.uniform(-1.0, 1.0)
    v = random.uniform(0.2, 0.6)
    A = random.uniform(0.5, 2.0)
    k = random.uniform(0.5, 1.5)

    pts = []
    for t in range(total_len):
        dt = t * 0.1
        x = x0 + v * dt
        y = y0 + A * math.sin(k * x)
        x += random.gauss(0.0, 0.01)
        y += random.gauss(0.0, 0.01)
        pts.append([x, y])
    return torch.tensor(pts, dtype=torch.float32)


def generate_zigzag(total_len: int) -> torch.Tensor:
    """Zigzag 之字形轨迹"""
    x0 = random.uniform(-4.0, -2.0)
    y0 = random.uniform(-1.0, 1.0)
    v = random.uniform(0.3, 0.8)
    seg_len = total_len // 3  # 三段

    dirs = [1.0, -1.0, 1.0]  # y 正负交替
    pts = []
    cur_x = x0
    cur_y = y0
    for i in range(total_len):
        dt = 0.1
        seg_idx = min(i // seg_len, 2)
        dy_sign = dirs[seg_idx]
        cur_x += v * dt
        cur_y += dy_sign * v * dt * 0.5  # y 变化比 x 小一点

        x = cur_x + random.gauss(0.0, 0.01)
        y = cur_y + random.gauss(0.0, 0.01)
        pts.append([x, y])
    return torch.tensor(pts, dtype=torch.float32)


def generate_figure_eight(total_len: int) -> torch.Tensor:
    """8 字型轨迹：简单参数方程"""
    cx = random.uniform(-1.0, 1.0)
    cy = random.uniform(-1.0, 1.0)
    A = random.uniform(1.0, 2.0)
    omega = random.uniform(0.5, 1.0)

    pts = []
    for t in range(total_len):
        dt = t * 0.1
        x = cx + A * math.sin(omega * dt)
        y = cy + A * math.sin(2.0 * omega * dt)
        x += random.gauss(0.0, 0.01)
        y += random.gauss(0.0, 0.01)
        pts.append([x, y])
    return torch.tensor(pts, dtype=torch.float32)


TRAJ_FUNCS = [
    ("straight_const", generate_straight_constant),
    ("straight_accel", generate_straight_accel),
    ("circle_arc", generate_circle_arc),
    ("s_curve", generate_s_curve),
    ("zigzag", generate_zigzag),
    ("figure_eight", generate_figure_eight),
]


# ==================== Dataset 定义 ====================

class TrajectoryDataset(Dataset):
    def __init__(self, num_per_type: int, obs_len: int, pred_len: int):
        """
        num_per_type: 每种轨迹类型生成多少条样本
        obs_len:      观测长度
        pred_len:     预测长度
        """
        super().__init__()
        self.obs_len = obs_len
        self.pred_len = pred_len
        self.total_len = obs_len + pred_len

        self.samples: List[Tuple[torch.Tensor, torch.Tensor]] = []
        self._generate_all(num_per_type)

    def _generate_all(self, num_per_type: int):
        for name, fn in TRAJ_FUNCS:
            for _ in range(num_per_type):
                traj = fn(self.total_len)  # [T, 2]
                obs = traj[: self.obs_len, :]
                fut = traj[self.obs_len :, :]
                self.samples.append((obs, fut))

        random.shuffle(self.samples)  # 打乱一下

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        return self.samples[idx]


# ==================== 训练主逻辑 ====================

def train():
    set_seed(1234)

    # --------- 超参数 ---------
    obs_len = 8
    pred_len = 12
    batch_size = 128
    num_epochs = 1000
    lr = 1e-3

    # 每类多少样本
    num_per_type_train = 1500  # 6 类 → 9000 样本
    num_per_type_val = 300     # 6 类 → 1800 样本

    print("=== 数据规模配置 ===")
    print(f"轨迹类型数量: {len(TRAJ_FUNCS)}")
    print(f"训练集: 每类 {num_per_type_train} 条 → 总计 {len(TRAJ_FUNCS) * num_per_type_train}")
    print(f"验证集: 每类 {num_per_type_val} 条 → 总计 {len(TRAJ_FUNCS) * num_per_type_val}")
    print("======================\n")

    # --------- 构造数据集 ---------
    train_dataset = TrajectoryDataset(num_per_type_train, obs_len, pred_len)
    val_dataset = TrajectoryDataset(num_per_type_val, obs_len, pred_len)

    train_loader = DataLoader(
        train_dataset, batch_size=batch_size, shuffle=True, drop_last=False
    )
    val_loader = DataLoader(
        val_dataset, batch_size=batch_size, shuffle=False, drop_last=False
    )

    # --------- 模型 & 优化器 ---------
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    model = LSTMTrajectoryModel(
        input_dim=2,
        hidden_dim=64,
        num_layers=2,
        pred_len=pred_len,
    ).to(device)

    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    criterion = nn.MSELoss()

    # --------- 模型保存路径 ---------
    save_dir = os.path.join(os.getcwd(), "models")
    os.makedirs(save_dir, exist_ok=True)
    save_path = os.path.join(save_dir, "trajectory_lstm.pth")

    best_val_loss = float("inf")

    # --------- 开始训练 ---------
    for epoch in range(1, num_epochs + 1):
        model.train()
        total_train_loss = 0.0
        num_train_batches = 0

        for obs, fut in train_loader:
            obs = obs.to(device)   # [B, obs_len, 2]
            fut = fut.to(device)   # [B, pred_len, 2]

            optimizer.zero_grad()
            pred = model(obs)      # [B, pred_len, 2]

            loss = criterion(pred, fut)
            loss.backward()
            optimizer.step()

            total_train_loss += loss.item()
            num_train_batches += 1

        avg_train_loss = total_train_loss / max(1, num_train_batches)

        # --------- 验证 ---------
        model.eval()
        total_val_loss = 0.0
        num_val_batches = 0

        with torch.no_grad():
            for obs, fut in val_loader:
                obs = obs.to(device)
                fut = fut.to(device)

                pred = model(obs)
                loss = criterion(pred, fut)

                total_val_loss += loss.item()
                num_val_batches += 1

        avg_val_loss = total_val_loss / max(1, num_val_batches)

        # 是否保存最优模型
        if avg_val_loss < best_val_loss:
            best_val_loss = avg_val_loss
            torch.save(
                {
                    "model_state_dict": model.state_dict(),
                    "obs_len": obs_len,
                    "pred_len": pred_len,
                    "hidden_dim": 64,
                    "num_layers": 2,
                },
                save_path,
            )
            improved = " (best)"
        else:
            improved = ""

        # 日志输出：每 20 轮详细打印一次，其余少量打印
        if epoch % 20 == 0 or epoch == 1:
            print(
                f"[Epoch {epoch:4d}/{num_epochs}] "
                f"train_loss={avg_train_loss:.4f}  "
                f"val_loss={avg_val_loss:.4f}  {improved}"
            )
        elif improved:
            # val 改善时也打印一条
            print(
                f"[Epoch {epoch:4d}] val improved → {avg_val_loss:.4f}  (保存模型)"
            )

    print("\n训练结束！")
    print(f"最佳验证损失: {best_val_loss:.6f}")
    print(f"最优模型已保存到: {save_path}")


if __name__ == "__main__":
    train()
