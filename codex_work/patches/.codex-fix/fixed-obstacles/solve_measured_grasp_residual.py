#!/usr/bin/env python3
"""Offline joint correction from one measured cube pose in link6."""

import numpy as np


ORIGINS = [(0, 0, 0.07), (0, 0, 0.05), (0, 0, 0.14),
           (0, 0, 0.22), (0, 0, 0.06), (0, 0, 0.06)]
AXES = [(0, 0, 1), (0, 1, 0), (0, 1, 0),
        (0, 0, 1), (0, -1, 0), (0, 0, 1)]


def link6_transform(q):
    transform = np.eye(4)
    for origin, axis, angle in zip(ORIGINS, AXES, q):
        translation = np.eye(4)
        translation[:3, 3] = origin
        axis = np.asarray(axis, dtype=float)
        skew = np.array([[0, -axis[2], axis[1]],
                         [axis[2], 0, -axis[0]],
                         [-axis[1], axis[0], 0]])
        rotation = np.eye(4)
        rotation[:3, :3] = (
            np.eye(3) + np.sin(angle) * skew
            + (1.0 - np.cos(angle)) * (skew @ skew))
        transform = transform @ translation @ rotation
    return transform


Q_OLD = np.array([0.0, 1.15253, 1.34909, 0.0, -0.39816, 0.0])
MEASURED_IN_LINK6 = np.array([-0.0329, 0.0161, 0.0738, 1.0])
TARGET_IN_LINK6 = np.array([0.0, 0.0, 0.045])
CUBE_IN_ARM_BASE = (link6_transform(Q_OLD) @ MEASURED_IN_LINK6)[:3]


def relative_position(q):
    return (
        np.linalg.inv(link6_transform(q))
        @ np.append(CUBE_IN_ARM_BASE, 1.0)
    )[:3]


def residual(q):
    cube_in_new_link6 = (
        np.linalg.inv(link6_transform(q))
        @ np.append(CUBE_IN_ARM_BASE, 1.0)
    )[:3]
    return cube_in_new_link6 - TARGET_IN_LINK6


# Damped numerical Gauss-Newton.  All six joints are available, while a
# posture regularizer and bounded trust step keep the solution close to the
# collision-tested pose.
q_new = Q_OLD.copy()
for _ in range(100):
    error = residual(q_new)
    if np.linalg.norm(error) < 1e-7:
        break
    eps = 1e-5
    jacobian = np.column_stack([
        (residual(q_new + np.eye(6)[joint] * eps) - error) / eps
        for joint in range(6)
    ])
    damping = 2e-3
    lhs = jacobian.T @ jacobian + damping * np.eye(6)
    rhs = -jacobian.T @ error - 2e-4 * (q_new - Q_OLD)
    step = np.linalg.solve(lhs, rhs)
    step_norm = np.linalg.norm(step)
    if step_norm > 0.08:
        step *= 0.08 / step_norm
    q_new = np.clip(q_new + step, -2.30, 2.30)

new_relative = relative_position(q_new)
print('CUBE_IN_ARM_BASE=', CUBE_IN_ARM_BASE.tolist())
print('Q_OLD=', Q_OLD.tolist())
print('Q_NEW=', q_new.tolist())
print('PREDICTED_RELATIVE=', new_relative.tolist())
print('PREDICTED_ERROR=',
      float(np.linalg.norm(new_relative - TARGET_IN_LINK6)))
