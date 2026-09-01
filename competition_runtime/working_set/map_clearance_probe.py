#!/usr/bin/env python3
import math
import sys

from PIL import Image


MAP_PATH = sys.argv[1]
ORIGIN_X = -15.6
ORIGIN_Y = -33.4
RESOLUTION = 0.05


def world_to_pixel(x, y, height):
    return (
        int((x - ORIGIN_X) / RESOLUTION),
        height - 1 - int((y - ORIGIN_Y) / RESOLUTION),
    )


image = Image.open(MAP_PATH).convert('L')
width, height = image.size
occupied = []
for py in range(height):
    for px in range(width):
        if image.getpixel((px, py)) < 89:
            occupied.append((px, py))

samples = []
start = (-5.83, 3.90)
goal = (float(sys.argv[2]), float(sys.argv[3]))
steps = 54
for index in range(steps + 1):
    ratio = index / steps
    samples.append((
        start[0] + (goal[0] - start[0]) * ratio,
        start[1] + (goal[1] - start[1]) * ratio,
    ))

minimum = (math.inf, None)
for x, y in samples:
    px, py = world_to_pixel(x, y, height)
    nearest = min(
        math.hypot(px - ox, py - oy) * RESOLUTION
        for ox, oy in occupied
    )
    if nearest < minimum[0]:
        minimum = (nearest, (x, y))

goal_px, goal_py = world_to_pixel(*goal, height)
goal_clearance = min(
    math.hypot(goal_px - ox, goal_py - oy) * RESOLUTION
    for ox, oy in occupied
)
print(f'goal={goal} static_clearance_m={goal_clearance:.3f}')
print(
    'segment_min_static_clearance_m='
    f'{minimum[0]:.3f} at ({minimum[1][0]:.3f}, {minimum[1][1]:.3f})'
)
