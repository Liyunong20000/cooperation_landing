#!/usr/bin/env python3
"""Generate a calibration chessboard with a configurable white border."""

import argparse
from pathlib import Path

import cv2
import numpy as np


def generate_chessboard(rows, cols, square_size, border_size):
    """Return a grayscale chessboard image as a NumPy array."""
    if min(rows, cols, square_size) <= 0 or border_size < 0:
        raise ValueError('rows, cols, and square size must be positive; border cannot be negative')

    height = rows * square_size + 2 * border_size
    width = cols * square_size + 2 * border_size
    image = np.full((height, width), 255, dtype=np.uint8)

    for row in range(rows):
        for col in range(cols):
            if (row + col) % 2:
                y0 = border_size + row * square_size
                x0 = border_size + col * square_size
                image[y0 : y0 + square_size, x0 : x0 + square_size] = 0
    return image


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='output PNG/JPEG path')
    parser.add_argument('--rows', type=int, default=9)
    parser.add_argument('--cols', type=int, default=7)
    parser.add_argument('--square-size', type=int, default=100, help='square size in pixels')
    parser.add_argument('--border-size', type=int, default=100, help='border size in pixels')
    parser.add_argument('--preview', action='store_true', help='open an OpenCV preview window')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        image = generate_chessboard(args.rows, args.cols, args.square_size, args.border_size)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), image):
        raise SystemExit(f'Failed to write {args.output}')
    if args.preview:
        cv2.imshow('Generated chessboard', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
