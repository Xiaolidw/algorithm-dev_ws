#!/usr/bin/env python3
"""Offline FK solve for a cube centered 0.32 m ahead of the mobile base."""

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


TARGET = np.array([0.32, 0.0, -0.1494])
ORIGINS = [
    (0.0, 0.0, 0.07),
    (0.0, 0.0, 0.05),
    (0.0, 0.0, 0.14),
    (0.0, 0.0, 0.22),
    (0.0, 0.0, 0.06),
    (0.0, 0.0, 0.06),
]
AXES = [
    (0.0, 0.0, 1.0),
    (0.0, 1.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
    (0.0, -1.0, 0.0),
    (0.0, 0.0, 1.0),
]


def transform_translation(xyz):
    matrix = np.eye(4)
    matrix[:3, 3] = xyz
    return matrix


def transform_rotation(axis, angle):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_rotvec(
        np.asarray(axis, dtype=float) * angle).as_matrix()
    return matrix


def link6_transform(q):
    transform = np.eye(4)
    for origin, axis, angle in zip(ORIGINS, AXES, q):
        transform = (
            transform
            @ transform_translation(origin)
            @ transform_rotation(axis, angle)
        )
    return transform


def fingertip_center(q):
    transform = link6_transform(q)
    return (transform @ np.array([0.0, 0.0, 0.045, 1.0]))[:3]


seed = np.array([0.0, 1.2, 1.17, 0.0, -0.3, 0.0])


def residual(active):
    q = np.array([0.0, active[0], active[1], 0.0, active[2], 0.0])
    position_error = fingertip_center(q) - TARGET
    regularization = 0.015 * (active - seed[[1, 2, 4]])
    return np.concatenate((position_error, regularization))


result = least_squares(
    residual,
    seed[[1, 2, 4]],
    bounds=([-2.30, -2.57, -2.30], [2.30, 2.57, 2.30]),
    method='trf',
    ftol=1e-12,
    xtol=1e-12,
    gtol=1e-12,
    max_nfev=500,
)
solution = np.array([
    0.0, result.x[0], result.x[1], 0.0, result.x[2], 0.0,
])
print('TARGET=', TARGET.tolist())
print('SEED_CENTER=', fingertip_center(seed).tolist())
print('SOLUTION_Q=', solution.tolist())
print('SOLUTION_CENTER=', fingertip_center(solution).tolist())
print('POSITION_ERROR=', float(np.linalg.norm(fingertip_center(solution)-TARGET)))
