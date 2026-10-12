#!/usr/bin/env python3
"""Generate the reviewed Phase-5 rectangular arena occupancy image."""

from math import isclose
from pathlib import Path


RESOLUTION_M = 0.05
INTERIOR_LENGTH_M = 6.85
INTERIOR_WIDTH_M = 5.00
INTERIOR_COLUMNS = 137
INTERIOR_ROWS = 100
BOUNDARY_CELLS = 1
IMAGE_COLUMNS = INTERIOR_COLUMNS + 2 * BOUNDARY_CELLS
IMAGE_ROWS = INTERIOR_ROWS + 2 * BOUNDARY_CELLS
FREE = 254
OCCUPIED = 0


def pixels() -> bytes:
    """Return top-to-bottom PGM rows with one occupied cell on each edge."""
    output = bytearray()
    for row in range(IMAGE_ROWS):
        for column in range(IMAGE_COLUMNS):
            boundary = row in (0, IMAGE_ROWS - 1) or column in (0, IMAGE_COLUMNS - 1)
            output.append(OCCUPIED if boundary else FREE)
    return bytes(output)


def main() -> None:
    if not isclose(INTERIOR_COLUMNS * RESOLUTION_M, INTERIOR_LENGTH_M):
        raise RuntimeError('arena length is not an exact number of cells')
    if not isclose(INTERIOR_ROWS * RESOLUTION_M, INTERIOR_WIDTH_M):
        raise RuntimeError('arena width is not an exact number of cells')
    repository_root = Path(__file__).resolve().parents[1]
    output = repository_root / 'lb_navigation' / 'maps' / 'arena.pgm'
    output.parent.mkdir(parents=True, exist_ok=True)
    header = (
        'P5\n'
        '# 6.85 m x 5.00 m free interior; 0.05 m occupied perimeter; column TBD\n'
        f'{IMAGE_COLUMNS} {IMAGE_ROWS}\n255\n'
    ).encode('ascii')
    output.write_bytes(header + pixels())
    print(output)


if __name__ == '__main__':
    main()
