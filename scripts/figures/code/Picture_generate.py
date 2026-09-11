#!/usr/bin/env python3
"""Resize an AprilTag image and place it on a printable white canvas."""

import argparse
from pathlib import Path

from PIL import Image


def create_printable_tag(
    input_path,
    output_path,
    tag_size=(2500, 2500),
    canvas_size=(2100, 2970),
    offset=(-200, -200),
):
    """Create a printable RGB canvas containing one resized tag image."""
    with Image.open(input_path) as image:
        resized = image.convert('RGB').resize(tag_size, Image.Resampling.NEAREST)

    canvas = Image.new('RGB', canvas_size, 'white')
    canvas.paste(resized, offset)
    canvas.save(output_path)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path, help='source AprilTag image')
    parser.add_argument('output', type=Path, help='output image path')
    parser.add_argument('--tag-width', type=int, default=2500)
    parser.add_argument('--tag-height', type=int, default=2500)
    parser.add_argument('--canvas-width', type=int, default=2100)
    parser.add_argument('--canvas-height', type=int, default=2970)
    parser.add_argument('--offset-x', type=int, default=-200)
    parser.add_argument('--offset-y', type=int, default=-200)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if min(args.tag_width, args.tag_height, args.canvas_width, args.canvas_height) <= 0:
        raise SystemExit('Image dimensions must be positive integers.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    create_printable_tag(
        args.input,
        args.output,
        tag_size=(args.tag_width, args.tag_height),
        canvas_size=(args.canvas_width, args.canvas_height),
        offset=(args.offset_x, args.offset_y),
    )


if __name__ == '__main__':
    main()
