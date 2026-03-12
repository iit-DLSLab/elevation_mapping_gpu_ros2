from __future__ import annotations

from pathlib import Path

import numpy as np
import cupy as cp

from elevation_mapping_cupy import ElevationMap, Parameter


def test_cupy_cuda_is_available():
    # Fail loudly: this repo targets CUDA + CuPy.
    n = cp.cuda.runtime.getDeviceCount()
    assert n >= 1


def test_kernels_compile_and_one_update_step_runs():
    # .../elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_cupy/tests/test_kernel_compile_smoke.py
    # parents[2] = ROS package root (contains config/).
    root = Path(__file__).resolve().parents[2]
    p = Parameter(
        use_chainer=False,
        weight_file=str(root / "config" / "core" / "weights.dat"),
        plugin_config_file=str(root / "config" / "core" / "plugin_config.yaml"),
    )

    # Keep map tiny so the smoke test stays fast.
    p.resolution = 0.2
    p.map_length = 4.0
    p.update()

    emap = ElevationMap(p)

    # A few points on a plane in the sensor frame.
    pts = np.array(
        [
            [1.0, 0.0, 0.0],
            [1.0, 0.5, 0.0],
            [1.0, -0.5, 0.0],
            [2.0, 0.0, 0.0],
            [2.0, 0.5, 0.0],
            [2.0, -0.5, 0.0],
        ],
        dtype=np.float32,
    )
    R = np.eye(3, dtype=np.float32)
    t = np.zeros(3, dtype=np.float32)

    # Should run without exceptions (kernels + traversability torch filter).
    emap.input_pointcloud(pts, ["x", "y", "z"], R, t, 0.0, 0.0)
    emap.update_variance()
    emap.update_time()
