"""Contract checks for the initial static arena raster and metadata."""

from pathlib import Path
import re


PACKAGE_ROOT = Path(__file__).parents[1]
MAP_YAML = PACKAGE_ROOT / 'maps' / 'arena.yaml'
MAP_IMAGE = PACKAGE_ROOT / 'maps' / 'arena.pgm'


def read_pgm(path):
    with path.open('rb') as stream:
        assert stream.readline().strip() == b'P5'
        line = stream.readline()
        while line.startswith(b'#'):
            line = stream.readline()
        columns, rows = (int(value) for value in line.split())
        assert int(stream.readline()) == 255
        pixels = stream.read()
    assert len(pixels) == columns * rows
    return columns, rows, pixels


def test_metadata_records_exact_geometry():
    metadata = MAP_YAML.read_text(encoding='utf-8')
    assert re.search(r'^image: arena\.pgm$', metadata, re.MULTILINE)
    assert re.search(r'^resolution: 0\.05$', metadata, re.MULTILINE)
    assert re.search(r'^origin: \[-0\.05, -0\.05, 0\.0\]$', metadata, re.MULTILINE)
    assert 'mode: trinary' in metadata
    assert 'occupied_thresh: 0.65' in metadata
    assert 'free_thresh: 0.196' in metadata
    assert 'center column remains TBD' in metadata


def test_map_has_one_cell_boundary_and_exact_free_interior():
    columns, rows, data = read_pgm(MAP_IMAGE)
    assert (columns, rows) == (139, 102)
    grid = [data[row * columns: (row + 1) * columns] for row in range(rows)]
    assert all(value == 0 for value in grid[0])
    assert all(value == 0 for value in grid[-1])
    assert all(row[0] == 0 and row[-1] == 0 for row in grid)
    interior = [value for row in grid[1:-1] for value in row[1:-1]]
    assert len(interior) == 137 * 100
    assert set(interior) == {254}


def test_image_rows_follow_ros_positive_y_convention_for_symmetric_boundary():
    columns, rows, data = read_pgm(MAP_IMAGE)
    top_inner = data[columns + 1: 2 * columns - 1]
    bottom_inner_start = (rows - 2) * columns + 1
    bottom_inner = data[bottom_inner_start: bottom_inner_start + columns - 2]
    assert top_inner == bottom_inner == bytes([254]) * 137
