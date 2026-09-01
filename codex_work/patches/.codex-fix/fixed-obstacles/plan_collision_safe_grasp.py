#!/usr/bin/env python3
"""Offline geometric planning for a collision-safe arm approach.

This does not command the robot.  It uses the URDF link cylinders and the
known cube centre in arm_base_link to find a joint-space route that keeps all
forearm links away from the cube until the final gripper insertion.
"""

import math
import random
import numpy as np


ORIGINS = [(0, 0, .07), (0, 0, .05), (0, 0, .14),
           (0, 0, .22), (0, 0, .06), (0, 0, .06)]
AXES = [(0, 0, 1), (0, 1, 0), (0, 1, 0),
        (0, 0, 1), (0, -1, 0), (0, 0, 1)]
LENGTHS = [.10, .14, .22, .06, .06, .02]
RADII = [.03, .03, .03, .025, .03, .05]
CUBE = np.array([.3104, .0019, -.1484])
CUBE_RADIUS = .026
HOME = np.array([0.0, .6102, 1.2593, 0.0, -1.4931, 0.0])


def transform_translation(xyz):
    m = np.eye(4)
    m[:3, 3] = xyz
    return m


def transform_rotation(axis, angle):
    a = np.asarray(axis, dtype=float)
    skew = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]],
                     [-a[1], a[0], 0]])
    m = np.eye(4)
    m[:3, :3] = (np.eye(3) + math.sin(angle) * skew
                 + (1 - math.cos(angle)) * (skew @ skew))
    return m


def link_segments(q):
    m = np.eye(4)
    segments = []
    for i, (origin, axis, angle) in enumerate(zip(ORIGINS, AXES, q)):
        m = m @ transform_translation(origin) @ transform_rotation(axis, angle)
        # link 1 is centred on joint1; links 2..6 begin at their joint.
        if i == 0:
            a = (m @ np.array([0, 0, -.05, 1]))[:3]
            b = (m @ np.array([0, 0,  .05, 1]))[:3]
        else:
            a = m[:3, 3]
            b = (m @ np.array([0, 0, LENGTHS[i], 1]))[:3]
        segments.append((a, b, RADII[i]))
    return segments


def segment_distance(point, a, b):
    v = b - a
    t = np.clip(np.dot(point - a, v) / max(np.dot(v, v), 1e-12), 0, 1)
    return np.linalg.norm(point - (a + t * v))


def clearance(q, final=False):
    # link6 is intentionally allowed only at final insertion; all other
    # links must remain beyond the cube plus a 2 cm safety margin.
    value = float('inf')
    for index, (a, b, radius) in enumerate(link_segments(q)):
        if final and index == 5:
            continue
        value = min(value, segment_distance(CUBE, a, b) - radius - CUBE_RADIUS)
    return value


def clearance_detail(q):
    return [segment_distance(CUBE, a, b) - radius - CUBE_RADIUS
            for a, b, radius in link_segments(q)]


def edge_safe(a, b):
    steps = max(2, int(np.linalg.norm(b - a) / .025))
    return min(clearance(a + (b - a) * (i / steps))
               for i in range(steps + 1)) > .02


def solve_target(target, seed):
    q = seed.copy()
    for _ in range(250):
        # link6 tip / intended cube centre is 4.5cm along +z from link6.
        m = np.eye(4)
        for origin, axis, angle in zip(ORIGINS, AXES, q):
            m = m @ transform_translation(origin) @ transform_rotation(axis, angle)
        centre = (m @ np.array([0, 0, .045, 1]))[:3]
        error = centre - target
        if np.linalg.norm(error) < 2e-4:
            break
        eps = 1e-5
        jac = []
        for j in range(6):
            q2 = q.copy(); q2[j] += eps
            m2 = np.eye(4)
            for origin, axis, angle in zip(ORIGINS, AXES, q2):
                m2 = m2 @ transform_translation(origin) @ transform_rotation(axis, angle)
            jac.append(((m2 @ np.array([0, 0, .045, 1]))[:3] - centre) / eps)
        jac = np.column_stack(jac)
        step = np.linalg.solve(jac.T @ jac + .003 * np.eye(6), -jac.T @ error)
        n = np.linalg.norm(step)
        if n > .06:
            step *= .06 / n
        q = np.clip(q + step, -2.30, 2.30)
    return q


FINAL = solve_target(CUBE, np.array([0, 1.15, 1.35, 0, -.4, 0]))
PRE = solve_target(CUBE + np.array([0, 0, .12]), FINAL)
KNOWN_FINAL = np.array([0.0, 1.15253, 1.34909, 0.0, -0.39816, 0.0])

print('HOME_CLEARANCE', clearance(HOME))
print('PRE', PRE.tolist(), 'CLEARANCE', clearance(PRE))
print('FINAL', FINAL.tolist(), 'CLEARANCE_WITHOUT_LINK6', clearance(FINAL, final=True))
print('KNOWN_FINAL', KNOWN_FINAL.tolist(),
      'CLEARANCE_WITHOUT_LINK6', clearance(KNOWN_FINAL, final=True))
print('FINAL_LINK_CLEARANCES', clearance_detail(FINAL))
print('KNOWN_LINK_CLEARANCES', clearance_detail(KNOWN_FINAL))
print('HOME_TO_PRE_SAFE', edge_safe(HOME, PRE))
print('PRE_TO_FINAL_SAFE', edge_safe(PRE, FINAL))

# Find one raised waypoint that is safe on both interpolated legs by sampling
# a bounded region near the folded-safe home posture.
rng = random.Random(20260830)
best = None
for _ in range(1500):
    candidate = HOME + np.array([
        rng.uniform(-.35, .35), rng.uniform(-.8, .7),
        rng.uniform(-.8, .8), rng.uniform(-.5, .5),
        rng.uniform(-.7, .9), rng.uniform(-.5, .5),
    ])
    candidate = np.clip(candidate, -2.30, 2.30)
    c = clearance(candidate)
    if c <= .02 or not edge_safe(HOME, candidate) or not edge_safe(candidate, PRE):
        continue
    score = min(c, clearance(PRE)) - .01 * np.linalg.norm(candidate - HOME)
    if best is None or score > best[0]:
        best = (score, candidate)

if best is None:
    print('NO_SAFE_WAYPOINT')
else:
    waypoint = best[1]
    print('WAYPOINT', waypoint.tolist(), 'CLEARANCE', clearance(waypoint))
    print('HOME_TO_WAYPOINT_SAFE', edge_safe(HOME, waypoint))
    print('WAYPOINT_TO_PRE_SAFE', edge_safe(waypoint, PRE))
