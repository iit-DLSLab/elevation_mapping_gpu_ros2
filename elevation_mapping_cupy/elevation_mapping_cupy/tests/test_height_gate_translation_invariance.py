import cupy as cp

from elevation_mapping_cupy.kernels.custom_kernels import map_utils


def _is_valid_probe_kernel():
    return cp.ElementwiseKernel(
        in_params="raw U px, raw U py, raw U pz, raw U sx, raw U sy, raw U sz",
        out_params="raw int8 out",
        preamble=map_utils(
            resolution=0.04,
            width=200,
            height=200,
            sensor_noise_factor=0.03,
            min_valid_distance=0.1,
            max_height_range=10.5,
            ramped_height_range_a=0.3,
            ramped_height_range_b=1.0,
            ramped_height_range_c=0.2,
        ),
        operation="out[i] = is_valid(px[i], py[i], pz[i], sx[i], sy[i], sz[i]) ? 1 : 0;",
        name="probe_is_valid_kernel",
    )


def _eval_is_valid(kernel, point_xyz, sensor_xyz):
    px = cp.asarray([point_xyz[0]], dtype=cp.float32)
    py = cp.asarray([point_xyz[1]], dtype=cp.float32)
    pz = cp.asarray([point_xyz[2]], dtype=cp.float32)
    sx = cp.asarray([sensor_xyz[0]], dtype=cp.float32)
    sy = cp.asarray([sensor_xyz[1]], dtype=cp.float32)
    sz = cp.asarray([sensor_xyz[2]], dtype=cp.float32)
    out = cp.zeros((1,), dtype=cp.int8)
    kernel(px, py, pz, sx, sy, sz, out, size=1)
    return int(out[0].item())


def test_height_gate_is_invariant_to_global_xy_translation():
    kernel = _is_valid_probe_kernel()
    # Relative geometry is identical in both cases: point is 2m ahead and 0.6m above sensor.
    # This should remain INVALID in both cases for the default ramp parameters.
    v_near_origin = _eval_is_valid(kernel, point_xyz=(2.0, 0.0, 0.6), sensor_xyz=(0.0, 0.0, 0.0))
    v_shifted_odom = _eval_is_valid(kernel, point_xyz=(22.0, 0.0, 0.6), sensor_xyz=(20.0, 0.0, 0.0))

    assert v_near_origin == v_shifted_odom
    assert v_near_origin == 0
