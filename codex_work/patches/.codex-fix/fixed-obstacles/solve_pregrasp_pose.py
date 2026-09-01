#!/usr/bin/env python3
"""Solve a collision-free waypoint 12 cm above the measured cube."""

import numpy as np

from solve_measured_grasp_residual import CUBE_IN_ARM_BASE, link6_transform


Q_FINAL = np.array([0.04452, 1.40105, 1.06118, 0.00311, -0.23730, 0.0])
TARGET = CUBE_IN_ARM_BASE + np.array([0.0, 0.0, 0.12])


def center(q):
    return (link6_transform(q) @ np.array([0.0, 0.0, 0.045, 1.0]))[:3]


q = Q_FINAL.copy()
for _ in range(150):
    error = center(q) - TARGET
    if np.linalg.norm(error) < 1e-6:
        break
    eps = 1e-5
    jacobian = np.column_stack([
        (center(q + np.eye(6)[joint] * eps) - center(q)) / eps
        for joint in range(6)
    ])
    lhs = jacobian.T @ jacobian + 3e-3 * np.eye(6)
    rhs = -jacobian.T @ error - 2e-4 * (q - Q_FINAL)
    step = np.linalg.solve(lhs, rhs)
    norm = np.linalg.norm(step)
    if norm > 0.06:
        step *= 0.06 / norm
    q = np.clip(q + step, -2.30, 2.30)

print('TARGET_CENTER=', TARGET.tolist())
print('Q_PREGRASP=', q.tolist())
print('SOLVED_CENTER=', center(q).tolist())
print('ERROR=', float(np.linalg.norm(center(q) - TARGET)))
