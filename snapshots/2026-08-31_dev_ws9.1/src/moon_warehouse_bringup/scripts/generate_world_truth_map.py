#!/usr/bin/env python3
"""Generate the static Nav2 map from the wall geometry Gazebo actually loads.

The saved ``<state>`` section in the competition world overrides several model
poses.  The old hand-edited PGM still contained walls from an earlier layout,
so Nav2 stopped at obstacles that did not exist in Gazebo.  This generator uses
the world-state link poses (world coordinates) and the matching collision-box
sizes, making the static map deterministic and reproducible.

Only permanent room walls belong here.  Cargo cubes, placement-zone occupancy,
fixed stones and moving obstacles are supplied by their runtime layers.
"""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def parse_pose(text):
    values = [float(value) for value in (text or '').split()]
    if len(values) != 6:
        raise ValueError(f'Expected six pose values, got {text!r}')
    return values


def find_named(elements, name):
    for element in elements:
        if element.get('name') == name:
            return element
    raise ValueError(f'Cannot find {name!r}')


def load_world_walls(world_path, model_name='officeroom'):
    root = ET.parse(world_path).getroot()
    world = root.find('world')
    if world is None:
        raise ValueError('SDF has no <world> element')

    model = find_named(world.findall('model'), model_name)
    sizes = {}
    for link in model.findall('link'):
        collision = link.find('collision')
        size = (collision.findtext('geometry/box/size')
                if collision is not None else None)
        if size:
            values = [float(value) for value in size.split()]
            if len(values) >= 2:
                sizes[link.get('name')] = (values[0], values[1])

    state = world.find('state')
    if state is None:
        raise ValueError('SDF has no saved <state>; actual link poses unknown')
    state_model = find_named(state.findall('model'), model_name)
    walls = []
    for link in state_model.findall('link'):
        name = link.get('name', '')
        if name not in sizes or not name.startswith('Wall_'):
            continue
        x, y, _z, _roll, _pitch, yaw = parse_pose(link.findtext('pose'))
        size_x, size_y = sizes[name]
        walls.append((name, x, y, yaw, size_x, size_y))
    if not walls:
        raise ValueError(f'No wall links found for model {model_name!r}')
    return walls


def generate_map(walls, output_path, resolution, origin_x, origin_y,
                 width, height, raster_margin):
    pixels = bytearray([254]) * (width * height)
    occupied = 0
    for _name, cx, cy, yaw, size_x, size_y in walls:
        half_x = size_x * 0.5 + raster_margin
        half_y = size_y * 0.5 + raster_margin
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        extent_x = abs(cosine) * half_x + abs(sine) * half_y
        extent_y = abs(sine) * half_x + abs(cosine) * half_y
        min_x = max(0, int(math.floor(
            (cx - extent_x - origin_x) / resolution)))
        max_x = min(width - 1, int(math.ceil(
            (cx + extent_x - origin_x) / resolution)))
        min_y = max(0, int(math.floor(
            (cy - extent_y - origin_y) / resolution)))
        max_y = min(height - 1, int(math.ceil(
            (cy + extent_y - origin_y) / resolution)))
        for grid_y in range(min_y, max_y + 1):
            wy = origin_y + (grid_y + 0.5) * resolution
            image_y = height - 1 - grid_y
            for grid_x in range(min_x, max_x + 1):
                wx = origin_x + (grid_x + 0.5) * resolution
                dx, dy = wx - cx, wy - cy
                local_x = cosine * dx + sine * dy
                local_y = -sine * dx + cosine * dy
                if abs(local_x) <= half_x and abs(local_y) <= half_y:
                    index = image_y * width + grid_x
                    if pixels[index] != 0:
                        pixels[index] = 0
                        occupied += 1

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('wb') as stream:
        stream.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
        stream.write(pixels)
    return occupied


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('world')
    parser.add_argument('output')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--origin-x', type=float, default=-15.6)
    parser.add_argument('--origin-y', type=float, default=-33.4)
    parser.add_argument('--width', type=int, default=658)
    parser.add_argument('--height', type=int, default=822)
    parser.add_argument(
        '--raster-margin', type=float, default=0.025,
        help='Half-cell wall rasterization margin in metres')
    args = parser.parse_args()
    walls = load_world_walls(args.world)
    occupied = generate_map(
        walls, args.output, args.resolution, args.origin_x, args.origin_y,
        args.width, args.height, args.raster_margin)
    print(f'Generated {args.output}: {len(walls)} walls, '
          f'{occupied} occupied cells')


if __name__ == '__main__':
    main()
