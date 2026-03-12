import cupy as cp

from elevation_mapping_cupy.kernels.custom_kernels import map_utils


def _probe_get_idx_kernel(axis: str, width: int, height: int, resolution: float):
    if axis not in ("x", "y"):
        raise ValueError(f"Unsupported axis '{axis}'. Expected 'x' or 'y'.")
    op = "out[i] = get_x_idx(coords[i], center[0]);" if axis == "x" else "out[i] = get_y_idx(coords[i], center[0]);"
    return cp.ElementwiseKernel(
        in_params="raw U coords, raw U center",
        out_params="raw int32 out",
        preamble=map_utils(
            resolution=resolution,
            width=width,
            height=height,
            sensor_noise_factor=0.0,
            min_valid_distance=0.0,
            max_height_range=1000.0,
            ramped_height_range_a=0.0,
            ramped_height_range_b=0.0,
            ramped_height_range_c=1000.0,
        ),
        operation=op,
        name=f"probe_get_{axis}_idx_kernel",
    )


def _probe_get_idx_and_inside_kernel(width: int, height: int, resolution: float):
    return cp.ElementwiseKernel(
        in_params="raw U px, raw U py, raw U center_x, raw U center_y",
        out_params="raw int32 out_idx, raw int8 out_inside",
        preamble=map_utils(
            resolution=resolution,
            width=width,
            height=height,
            sensor_noise_factor=0.0,
            min_valid_distance=0.0,
            max_height_range=1000.0,
            ramped_height_range_a=0.0,
            ramped_height_range_b=0.0,
            ramped_height_range_c=1000.0,
        ),
        operation=(
            "int idx = get_idx(px[i], py[i], center_x[0], center_y[0]); "
            "out_idx[i] = idx; "
            "out_inside[i] = is_inside(idx) ? 1 : 0;"
        ),
        name="probe_get_idx_and_inside_kernel",
    )


def test_get_x_idx_rounds_left_of_boundary_before_clamp():
    kernel = _probe_get_idx_kernel(axis="x", width=200, height=200, resolution=1.0)
    coords = cp.asarray([-100.2], dtype=cp.float32)
    center = cp.asarray([0.0], dtype=cp.float32)
    out = cp.zeros((1,), dtype=cp.int32)
    kernel(coords, center, out, size=1)
    assert int(out[0].item()) == -1


def test_get_y_idx_at_center_matches_middle_cell():
    kernel = _probe_get_idx_kernel(axis="y", width=200, height=200, resolution=1.0)
    coords = cp.asarray([0.0], dtype=cp.float32)
    center = cp.asarray([0.0], dtype=cp.float32)
    out = cp.zeros((1,), dtype=cp.int32)
    kernel(coords, center, out, size=1)
    assert int(out[0].item()) == 100


def test_get_idx_clamps_far_out_of_range_to_border():
    width = 200
    height = 200
    kernel = _probe_get_idx_and_inside_kernel(width=width, height=height, resolution=1.0)

    px = cp.asarray([-1.0e6, 1.0e6], dtype=cp.float32)
    py = cp.asarray([0.0, 0.0], dtype=cp.float32)
    center_x = cp.asarray([0.0], dtype=cp.float32)
    center_y = cp.asarray([0.0], dtype=cp.float32)
    out_idx = cp.zeros((2,), dtype=cp.int32)
    out_inside = cp.zeros((2,), dtype=cp.int8)

    kernel(px, py, center_x, center_y, out_idx, out_inside, size=2)

    idx_left = int(out_idx[0].item())
    idx_right = int(out_idx[1].item())
    # Row-major decode: idx = width * row + col
    col_left = idx_left % width
    col_right = idx_right % width

    assert col_left == 0
    assert col_right == width - 1
    assert int(out_inside[0].item()) == 0
    assert int(out_inside[1].item()) == 0


def test_is_inside_marks_border_false_and_near_border_true():
    width = 200
    height = 200
    kernel = _probe_get_idx_and_inside_kernel(width=width, height=height, resolution=1.0)

    px = cp.asarray([-100.0, -99.0, 98.0, 99.0], dtype=cp.float32)
    py = cp.asarray([0.0, 0.0, 0.0, 0.0], dtype=cp.float32)
    center_x = cp.asarray([0.0], dtype=cp.float32)
    center_y = cp.asarray([0.0], dtype=cp.float32)
    out_idx = cp.zeros((4,), dtype=cp.int32)
    out_inside = cp.zeros((4,), dtype=cp.int8)

    kernel(px, py, center_x, center_y, out_idx, out_inside, size=4)

    # x=-100 -> left border col=0: outside
    assert int(out_inside[0].item()) == 0
    # x=-99 -> next col=1: inside
    assert int(out_inside[1].item()) == 1
    # x=98 -> col=198: inside
    assert int(out_inside[2].item()) == 1
    # x=99 -> right border col=199: outside
    assert int(out_inside[3].item()) == 0
