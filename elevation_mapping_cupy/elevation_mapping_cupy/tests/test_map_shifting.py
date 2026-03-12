"""
Tests for map shifting functionality.

These tests verify that the map shifts correctly when the robot moves,
ensuring X movement affects X axis and Y movement affects Y axis.

This test suite was created to prevent regression of the axis swap bug
where forward robot movement (X) was incorrectly causing sideways map shift (Y).
"""

import pytest
import numpy as np
import cupy as cp
from pathlib import Path
from elevation_mapping_cupy import parameter, elevation_mapping

# Get absolute paths to config files
_TEST_DIR = Path(__file__).parent
_CONFIG_DIR = _TEST_DIR.parent.parent / "config" / "core"


@pytest.fixture
def elmap_shift():
    """Create a minimal elevation map for shift testing."""
    p = parameter.Parameter(
        use_chainer=False,
        weight_file=str(_CONFIG_DIR / "weights.dat"),
        plugin_config_file=str(_CONFIG_DIR / "plugin_config.yaml"),
    )
    # Use default resolution (0.1m) and map_length (20m) -> ~200x200 cells
    p.update()
    e = elevation_mapping.ElevationMap(p)
    e.clear()  # Start with clean map
    return e


class TestShiftMapXY:
    """Tests for the shift_map_xy function."""

    def test_shift_x_only_affects_columns(self, elmap_shift):
        """
        X-only shift should only affect columns (axis 2), not rows (axis 1).

        When we shift by [x=5, y=0], the marker at (row=100, col=100) should:
        - Move to col=105 (shifted in column/X direction)
        - Stay at row=100 (NOT shifted in row/Y direction)
        """
        center_idx = elmap_shift.cell_n // 2

        # Place a marker at the center
        elmap_shift.elevation_map[0, center_idx, center_idx] = 1.0

        # Shift by [5, 0] -> X=5 pixels, Y=0 pixels
        shift_amount = 5
        elmap_shift.shift_map_xy(cp.array([shift_amount, 0], dtype=cp.float32))

        # After X shift, marker should have moved in column direction
        # cp.roll with positive shift moves elements to higher indices
        new_col = center_idx + shift_amount

        # Marker should be at new column position, same row
        assert float(elmap_shift.elevation_map[0, center_idx, new_col]) == 1.0, \
            f"Marker should be at (row={center_idx}, col={new_col}) after X shift"

        # Marker should NOT be at swapped position (wrong axis)
        new_row_wrong = center_idx + shift_amount
        assert float(elmap_shift.elevation_map[0, new_row_wrong, center_idx]) == 0.0, \
            f"Marker should NOT be at (row={new_row_wrong}, col={center_idx}) - X shift should not affect rows"

    def test_shift_y_only_affects_rows(self, elmap_shift):
        """
        Y-only shift should only affect rows (axis 1), not columns (axis 2).

        When we shift by [x=0, y=5], the marker at (row=100, col=100) should:
        - Move to row=105 (shifted in row/Y direction)
        - Stay at col=100 (NOT shifted in column/X direction)
        """
        center_idx = elmap_shift.cell_n // 2

        # Place a marker at the center
        elmap_shift.elevation_map[0, center_idx, center_idx] = 1.0

        # Shift by [0, 5] -> X=0 pixels, Y=5 pixels
        shift_amount = 5
        elmap_shift.shift_map_xy(cp.array([0, shift_amount], dtype=cp.float32))

        # After Y shift, marker should have moved in row direction
        new_row = center_idx + shift_amount

        # Marker should be at new row position, same column
        assert float(elmap_shift.elevation_map[0, new_row, center_idx]) == 1.0, \
            f"Marker should be at (row={new_row}, col={center_idx}) after Y shift"

        # Marker should NOT be at swapped position (wrong axis)
        new_col_wrong = center_idx + shift_amount
        assert float(elmap_shift.elevation_map[0, center_idx, new_col_wrong]) == 0.0, \
            f"Marker should NOT be at (row={center_idx}, col={new_col_wrong}) - Y shift should not affect columns"

    def test_diagonal_shift(self, elmap_shift):
        """Diagonal shift should affect both axes correctly."""
        center_idx = elmap_shift.cell_n // 2

        # Place a marker at the center
        elmap_shift.elevation_map[0, center_idx, center_idx] = 1.0

        # Shift by [3, 7] -> X=3, Y=7
        shift_x, shift_y = 3, 7
        elmap_shift.shift_map_xy(cp.array([shift_x, shift_y], dtype=cp.float32))

        # Marker should be at (row + y, col + x)
        expected_row = center_idx + shift_y
        expected_col = center_idx + shift_x

        assert float(elmap_shift.elevation_map[0, expected_row, expected_col]) == 1.0, \
            f"Marker should be at (row={expected_row}, col={expected_col}) after diagonal shift"

    def test_negative_shift(self, elmap_shift):
        """Negative shifts should work correctly."""
        center_idx = elmap_shift.cell_n // 2

        # Place a marker at the center
        elmap_shift.elevation_map[0, center_idx, center_idx] = 1.0

        # Shift by [-5, -3] -> X=-5, Y=-3
        shift_x, shift_y = -5, -3
        elmap_shift.shift_map_xy(cp.array([shift_x, shift_y], dtype=cp.float32))

        # Marker should move to lower indices
        expected_row = center_idx + shift_y
        expected_col = center_idx + shift_x

        assert float(elmap_shift.elevation_map[0, expected_row, expected_col]) == 1.0, \
            f"Marker should be at (row={expected_row}, col={expected_col}) after negative shift"

    def test_zero_shift_no_change(self, elmap_shift):
        """Zero shift should not modify the map."""
        center_idx = elmap_shift.cell_n // 2

        # Place a marker
        elmap_shift.elevation_map[0, center_idx, center_idx] = 1.0
        original_map = elmap_shift.elevation_map.copy()

        # Shift by [0, 0]
        elmap_shift.shift_map_xy(cp.array([0, 0], dtype=cp.float32))

        # Map should be unchanged
        assert cp.allclose(elmap_shift.elevation_map, original_map), \
            "Zero shift should not modify the map"


class TestMoveTo:
    """Tests for the move_to function which uses shift_map_xy internally."""

    def test_move_to_x_positive(self, elmap_shift):
        """
        Robot moving in +X direction should shift map in -X direction.
        This makes the map appear to scroll backward as robot moves forward.
        """
        # Get initial center
        initial_center = cp.asnumpy(elmap_shift.center.copy())

        # Place a marker at the center of the map
        center_idx = elmap_shift.cell_n // 2
        elmap_shift.elevation_map[0, center_idx, center_idx] = 1.0

        # Move robot 1m in +X direction (10 cells at 0.1m resolution)
        move_distance = 1.0  # meters
        R = np.eye(3, dtype=np.float32)
        elmap_shift.move_to(np.array([move_distance, 0.0, 0.0], dtype=np.float32), R)

        # Map center should have updated
        new_center = cp.asnumpy(elmap_shift.center)
        assert new_center[0] > initial_center[0], \
            "Map center X should increase when robot moves +X"
        assert abs(new_center[1] - initial_center[1]) < 1e-6, \
            "Map center Y should not change for X-only movement"

    def test_move_to_y_positive(self, elmap_shift):
        """
        Robot moving in +Y direction should shift map in -Y direction.
        """
        initial_center = cp.asnumpy(elmap_shift.center.copy())

        # Move robot 1m in +Y direction
        move_distance = 1.0
        R = np.eye(3, dtype=np.float32)
        elmap_shift.move_to(np.array([0.0, move_distance, 0.0], dtype=np.float32), R)

        new_center = cp.asnumpy(elmap_shift.center)
        assert new_center[1] > initial_center[1], \
            "Map center Y should increase when robot moves +Y"
        assert abs(new_center[0] - initial_center[0]) < 1e-6, \
            "Map center X should not change for Y-only movement"

    def test_move_to_preserves_relative_data(self, elmap_shift):
        """
        After moving, data in the map should maintain its world position.
        A point at world position (1, 0) should remain visible after robot moves.
        """
        resolution = elmap_shift.resolution
        center_idx = elmap_shift.cell_n // 2

        # Place a marker 1m ahead of center in X
        offset_cells = int(1.0 / resolution)
        marker_col = center_idx + offset_cells
        elmap_shift.elevation_map[0, center_idx, marker_col] = 1.0

        # Move robot forward by 0.5m (less than the marker distance)
        R = np.eye(3, dtype=np.float32)
        elmap_shift.move_to(np.array([0.5, 0.0, 0.0], dtype=np.float32), R)

        # Marker should still be visible (shifted but within map)
        # The marker was at col = center + 10, after shifting -5, it should be at center + 5
        expected_new_col = marker_col - int(0.5 / resolution)

        assert float(elmap_shift.elevation_map[0, center_idx, expected_new_col]) == 1.0, \
            "Marker should maintain relative world position after robot movement"


class TestPadValue:
    """Tests for padding behavior after shifts."""

    def test_positive_x_shift_pads_left(self, elmap_shift):
        """After positive X shift, new cells should appear on the left (low column indices)."""
        # Fill entire elevation with 1.0
        elmap_shift.elevation_map[0, :, :] = 1.0

        # Shift positive X
        shift_amount = 10
        elmap_shift.shift_map_xy(cp.array([shift_amount, 0], dtype=cp.float32))

        # Left edge (low column indices) should be padded with 0
        assert cp.all(elmap_shift.elevation_map[0, :, :shift_amount] == 0.0), \
            "Left edge should be padded with 0 after positive X shift"

        # Right edge should still have data
        assert cp.any(elmap_shift.elevation_map[0, :, shift_amount:] != 0.0), \
            "Right side should still have data after positive X shift"

    def test_positive_y_shift_pads_top(self, elmap_shift):
        """After positive Y shift, new cells should appear at top (low row indices)."""
        # Fill entire elevation with 1.0
        elmap_shift.elevation_map[0, :, :] = 1.0

        # Shift positive Y
        shift_amount = 10
        elmap_shift.shift_map_xy(cp.array([0, shift_amount], dtype=cp.float32))

        # Top edge (low row indices) should be padded with 0
        assert cp.all(elmap_shift.elevation_map[0, :shift_amount, :] == 0.0), \
            "Top edge should be padded with 0 after positive Y shift"

        # Bottom should still have data
        assert cp.any(elmap_shift.elevation_map[0, shift_amount:, :] != 0.0), \
            "Bottom side should still have data after positive Y shift"


#
# Semantic/image fusion was intentionally removed from the supported surface of this repo.
# Keep map-shift tests focused on the elevation map core.
