#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import rosbag2_py
from geometry_msgs.msg import Pose
from rclpy.serialization import deserialize_message


TARGET_TOPICS = {
    '/moving_obstacle_1/current_pose': 'moving_obstacle_1',
    '/moving_obstacle_2/current_pose': 'moving_obstacle_2',
}


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Convert dynamic-obstacle Pose topics from rosbag2 to CSV.'
    )
    parser.add_argument(
        'bag_path',
        help='Path to the rosbag2 directory containing metadata.yaml.',
    )
    parser.add_argument(
        'output_csv',
        help='Output CSV file path.',
    )
    return parser.parse_args()


def open_bag(bag_path: Path):
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id='sqlite3',
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    return reader


def main():
    args = parse_arguments()

    bag_path = Path(args.bag_path).expanduser().resolve()
    output_csv = Path(args.output_csv).expanduser().resolve()

    metadata_path = bag_path / 'metadata.yaml'

    if not bag_path.is_dir():
        raise FileNotFoundError(f'Rosbag directory does not exist: {bag_path}')

    if not metadata_path.is_file():
        raise FileNotFoundError(
            f'metadata.yaml was not found in rosbag directory: {bag_path}'
        )

    output_csv.parent.mkdir(parents=True, exist_ok=True)

    reader = open_bag(bag_path)

    available_topics = {
        item.name: item.type
        for item in reader.get_all_topics_and_types()
    }

    print('Topics found in rosbag:')

    for topic_name, topic_type in available_topics.items():
        print(f'  {topic_name}: {topic_type}')

    missing_topics = [
        topic_name
        for topic_name in TARGET_TOPICS
        if topic_name not in available_topics
    ]

    if missing_topics:
        raise RuntimeError(
            f'Required topics are missing from rosbag: {missing_topics}'
        )

    first_timestamp_ns = None
    total_count = 0

    topic_counts = {
        obstacle_id: 0
        for obstacle_id in TARGET_TOPICS.values()
    }

    first_positions = {}
    last_positions = {}

    with output_csv.open('w', newline='', encoding='utf-8') as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                'bag_time_ns',
                'elapsed_sec',
                'obstacle_id',
                'x',
                'y',
                'z',
                'orientation_x',
                'orientation_y',
                'orientation_z',
                'orientation_w',
            ],
        )

        writer.writeheader()

        while reader.has_next():
            topic_name, serialized_data, timestamp_ns = reader.read_next()

            if topic_name not in TARGET_TOPICS:
                continue

            if first_timestamp_ns is None:
                first_timestamp_ns = timestamp_ns

            pose = deserialize_message(serialized_data, Pose)
            obstacle_id = TARGET_TOPICS[topic_name]

            elapsed_sec = (
                timestamp_ns - first_timestamp_ns
            ) / 1_000_000_000.0

            writer.writerow({
                'bag_time_ns': timestamp_ns,
                'elapsed_sec': f'{elapsed_sec:.9f}',
                'obstacle_id': obstacle_id,
                'x': f'{pose.position.x:.9f}',
                'y': f'{pose.position.y:.9f}',
                'z': f'{pose.position.z:.9f}',
                'orientation_x': f'{pose.orientation.x:.9f}',
                'orientation_y': f'{pose.orientation.y:.9f}',
                'orientation_z': f'{pose.orientation.z:.9f}',
                'orientation_w': f'{pose.orientation.w:.9f}',
            })

            position = (
                pose.position.x,
                pose.position.y,
                pose.position.z,
            )

            if obstacle_id not in first_positions:
                first_positions[obstacle_id] = position

            last_positions[obstacle_id] = position
            topic_counts[obstacle_id] += 1
            total_count += 1

    if total_count == 0:
        raise RuntimeError('No dynamic-obstacle Pose messages were extracted.')

    print()
    print(f'CSV written to: {output_csv}')
    print(f'Total Pose messages: {total_count}')

    for obstacle_id, count in topic_counts.items():
        print()
        print(f'{obstacle_id}:')
        print(f'  message count: {count}')
        print(f'  first position: {first_positions.get(obstacle_id)}')
        print(f'  last position:  {last_positions.get(obstacle_id)}')


if __name__ == '__main__':
    main()
