import pytest
from elevation_mapping_cupy.parameter import Parameter
from pathlib import Path


def test_parameter():
    # .../elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_cupy/tests/test_parameter.py
    # parents[2] = ROS package root (contains config/).
    root = Path(__file__).resolve().parents[2]
    param = Parameter(
        use_chainer=False,
        weight_file=str(root / "config" / "core" / "weights.dat"),
        plugin_config_file=str(root / "config" / "core" / "plugin_config.yaml"),
    )
    res = param.resolution
    param.set_value("resolution", 0.1)
    param.get_types()
    param.get_names()
    param.update()
    assert param.resolution == param.get_value("resolution")
    param.load_weights(param.weight_file)
