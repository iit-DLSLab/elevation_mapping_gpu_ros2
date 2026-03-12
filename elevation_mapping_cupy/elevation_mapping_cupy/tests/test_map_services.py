import numpy as np
from pathlib import Path

from elevation_mapping_cupy import parameter, elevation_mapping
from elevation_mapping_cupy.elevation_mapping import GridGeometry


def make_map(resolution=0.2, length=1.0):
    root = Path(__file__).resolve().parents[2]
    param = parameter.Parameter(
        use_chainer=False,
        weight_file=str(root / "config/core/weights.dat"),
        plugin_config_file=str(root / "config/core/plugin_config.yaml"),
    )
    param.resolution = resolution
    param.map_length = length
    param.update()
    return elevation_mapping.ElevationMap(param)


def build_geometry(map_obj):
    size = map_obj.cell_n - 2
    length = size * map_obj.resolution
    center = np.zeros(3, dtype=np.float32)
    orientation = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
    return GridGeometry(
        length_x=length,
        length_y=length,
        resolution=map_obj.resolution,
        center=center,
        orientation=orientation,
    )


def test_apply_masked_replace_updates_layer():
    emap = make_map()
    emap.elevation_map[2].fill(1.0)
    geometry = build_geometry(emap)
    size = emap.cell_n - 2
    layer = np.full((size, size), 1.5, dtype=np.float32)
    mask = np.ones((size, size), dtype=np.float32)
    mask[size // 2 :, :] = np.nan

    emap.apply_masked_replace({"elevation": layer}, mask, geometry)

    buffer = np.zeros((size, size), dtype=np.float32)
    emap.get_map_with_name_ref("elevation", buffer)

    assert np.isclose(buffer, 1.5, atol=1e-6).sum() == (size // 2) * size
    assert np.isclose(buffer, 0.0, atol=1e-6).sum() == (size - size // 2) * size


def test_set_full_map_overwrites_layers():
    emap = make_map()
    emap.elevation_map[2].fill(1.0)
    geometry = build_geometry(emap)
    size = emap.cell_n - 2

    elevation = np.full((size, size), 2.0, dtype=np.float32)
    variance = np.full((size, size), 0.25, dtype=np.float32)

    raw_layers = {
        "elevation": elevation,
        "variance": variance,
        "is_valid": np.ones((size, size), dtype=np.float32),
    }

    emap.set_full_map({}, raw_layers, geometry)

    buffer = np.zeros((size, size), dtype=np.float32)
    emap.get_map_with_name_ref("elevation", buffer)
    assert np.allclose(buffer, 2.0, atol=1e-6)

    emap.get_map_with_name_ref("variance", buffer)
    assert np.allclose(buffer, 0.25, atol=1e-6)
