# lstm_trajectory_model.py
# 简单的 LSTM 轨迹预测模型：输入观测序列 (x,y)，输出未来若干步的 (x,y)

from typing import Tuple

import torch
import torch.nn as nn


class LSTMTrajectoryModel(nn.Module):
    """
    输入:  (batch, obs_len, 2)   ->  观测到的历史轨迹 (x, y)
    输出:  (batch, pred_len, 2)  ->  未来预测的轨迹 (x, y)

    这里我们只做一个最基础的 many-to-many LSTM：
    - 把观测序列送入 LSTM 得到最后的隐状态 (h, c)
    - 用这个隐状态一步一步地展开预测 pred_len 步，将每步输出映射到 (x,y)
    """

    def __init__(
        self,
        input_dim: int = 2,      # 输入维度: (x, y)
        hidden_dim: int = 64,    # LSTM 隐层维度
        num_layers: int = 2,     # LSTM 层数
        output_dim: int = 2,     # 输出维度: (x, y)
        pred_len: int = 10,      # 预测步数（需要和训练脚本里保持一致）
    ):
        super().__init__()

        self.input_dim = input_dim
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers
        self.output_dim = output_dim
        self.pred_len = pred_len

        # LSTM 编码器
        self.lstm = nn.LSTM(
            input_size=input_dim,
            hidden_size=hidden_dim,
            num_layers=num_layers,
            batch_first=True,
        )

        # 将 LSTM 输出映射到 (x,y)
        self.output_layer = nn.Linear(hidden_dim, output_dim)

    def forward(self, obs_seq: torch.Tensor) -> torch.Tensor:
        """
        obs_seq: (batch, obs_len, 2)
        return:  (batch, pred_len, 2)
        """
        batch_size = obs_seq.size(0)

        # 先把观测序列丢进 LSTM，拿到最后的隐状态 (h, c)
        # lstm_out: (batch, obs_len, hidden_dim)，我们其实暂时不用
        lstm_out, (h, c) = self.lstm(obs_seq)

        # 用最后的隐状态作为“种子”，一步一步预测未来 pred_len 步
        preds = []
        # 为了简单，这里每一步的输入都用 0 向量（你之后可以改成用上一时刻的预测坐标）
        step_input = torch.zeros(batch_size, 1, self.input_dim, device=obs_seq.device)

        h_t = h
        c_t = c

        for _ in range(self.pred_len):
            out_t, (h_t, c_t) = self.lstm(step_input, (h_t, c_t))
            # out_t: (batch, 1, hidden_dim) -> (batch, 1, 2)
            coord_t = self.output_layer(out_t)      # (batch, 1, 2)
            preds.append(coord_t)
            # 如果想用自回归方式，可以把下面这行改成用 coord_t
            step_input = torch.zeros_like(step_input)

        # 拼成 (batch, pred_len, 2)
        preds = torch.cat(preds, dim=1)
        return preds
