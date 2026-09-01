import math
import sys

sys.path.insert(0, '/home/ros/dev_ws/src/moon_warehouse_bringup/scripts')
from generate_world_truth_map import load_world_walls


def rect(cx, cy, yaw, sx, sy):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    corners = [(-sx / 2, -sy / 2), (sx / 2, -sy / 2),
               (sx / 2, sy / 2), (-sx / 2, sy / 2)]
    return [(cx + cosine * dx - sine * dy,
             cy + sine * dx + cosine * dy) for dx, dy in corners]


def orient(a, b, c):
    return ((b[0] - a[0]) * (c[1] - a[1])
            - (b[1] - a[1]) * (c[0] - a[0]))


def inside(point, polygon):
    values = [orient(polygon[i], polygon[(i + 1) % len(polygon)], point)
              for i in range(len(polygon))]
    return all(value >= -1e-9 for value in values) or all(
        value <= 1e-9 for value in values)


def point_segment_distance(point, start, end):
    vx, vy = end[0] - start[0], end[1] - start[1]
    wx, wy = point[0] - start[0], point[1] - start[1]
    ratio = max(0.0, min(1.0, (vx * wx + vy * wy) / (vx * vx + vy * vy)))
    return math.hypot(point[0] - (start[0] + ratio * vx),
                      point[1] - (start[1] + ratio * vy))


def segment_distance(a, b, c, d):
    if orient(a, b, c) * orient(a, b, d) <= 0 and orient(c, d, a) * orient(c, d, b) <= 0:
        return 0.0
    return min(point_segment_distance(a, c, d), point_segment_distance(b, c, d),
               point_segment_distance(c, a, b), point_segment_distance(d, a, b))


def polygon_distance(first, second):
    if any(inside(point, second) for point in first) or any(
            inside(point, first) for point in second):
        return 0.0
    return min(segment_distance(first[i], first[(i + 1) % len(first)],
                                second[j], second[(j + 1) % len(second)])
               for i in range(len(first)) for j in range(len(second)))


world = '/home/ros/dev_ws/src/yzbot/mybot_description/worlds/competition_v2_scoring.world'
robot = rect(-3.04, -5.00, 0.0, 0.72, 0.48)
items = []
for name, x, y, yaw, sx, sy in load_world_walls(world):
    items.append((polygon_distance(robot, rect(x, y, yaw, sx, sy)),
                  'wall', name, x, y, sx, sy))

stones = [(-11.7361, 10.376), (-9.86873, 4.59948), (0.764529, 6.87658),
          (14.5365, 7.25535), (-3.6643, 2.16941), (5.45574, 2.04349),
          (7.19325, -1.71375), (0.605863, -2.01965), (-14.4058, -2.10305),
          (-11.6359, -5.34072), (11.7248, -5.10078), (-3.47257, -8.45059),
          (14.5027, -8.59174)]
for index, (x, y) in enumerate(stones):
    items.append((polygon_distance(robot, rect(x, y, 0.0, 1.0, 1.0)),
                  'stone', f'stone_{index}', x, y, 1.0, 1.0))

for item in sorted(items)[:12]:
    print(item)
