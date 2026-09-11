#!/usr/bin/env python3
"""Export all messages in a ROS 1 bag to a simple CSV file."""

import argparse
import csv
from pathlib import Path

import rosbag


def convert_bag_to_csv(bag_file, csv_file):
    """Write timestamp, topic, and stringified message columns."""
    with rosbag.Bag(str(bag_file), 'r') as bag, csv_file.open(
        'w', newline='', encoding='utf-8'
    ) as output:
        writer = csv.writer(output)
        writer.writerow(['timestamp', 'topic', 'message'])
        for topic, message, stamp in bag.read_messages():
            writer.writerow([stamp.to_sec(), topic, str(message)])


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path, help='input ROS bag')
    parser.add_argument('csv', type=Path, help='output CSV')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if not args.bag.is_file():
        raise SystemExit(f'Bag file does not exist: {args.bag}')
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    convert_bag_to_csv(args.bag, args.csv)


if __name__ == '__main__':
    main()
