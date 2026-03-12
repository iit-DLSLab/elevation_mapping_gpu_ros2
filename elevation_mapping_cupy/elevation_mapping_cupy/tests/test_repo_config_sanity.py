from __future__ import annotations

from pathlib import Path

import pytest
from ruamel.yaml import YAML


def _pkg_root() -> Path:
    # .../elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_cupy/tests/test_repo_config_sanity.py
    # parents[2] = ROS package root (contains config/).
    return Path(__file__).resolve().parents[2]


def _supported_config_files() -> list[Path]:
    cfg_root = _pkg_root() / "config"
    assert cfg_root.is_dir(), f"Missing config dir: {cfg_root}"
    yamls = sorted(cfg_root.rglob("*.yaml"))
    supported = []
    for p in yamls:
        # Keep experimental configs in the tree, but don't treat them as supported.
        if "experimental" in p.parts:
            continue
        supported.append(p)
    return supported


@pytest.mark.parametrize("path", _supported_config_files())
def test_supported_configs_are_ros2_clean(path: Path):
    text = path.read_text(encoding="utf-8")

    # No ROS1-style or non-standard substitutions in supported configs.
    banned = [
        "$(rospack find",
        "$(find_package_share",
        "$(find-pkg-share",
    ]
    for token in banned:
        assert token not in text, f"{path} contains banned substitution token: {token}"

    # Kill the old typo forever.
    assert "drift_compensation_variance_inler" not in text, (
        f"{path} contains deprecated typo key drift_compensation_variance_inler"
    )

    # Parseable YAML.
    yaml = YAML(typ="safe")
    data = yaml.load(text) or {}
    assert isinstance(data, dict), f"{path} must parse to a dict, got {type(data)}"

    # Supported configs must not reference removed semantic/image features.
    def _walk(obj):
        if isinstance(obj, dict):
            for k, v in obj.items():
                yield k, v
                yield from _walk(v)
        elif isinstance(obj, list):
            for v in obj:
                yield from _walk(v)

    for k, v in _walk(data):
        # If someone keeps an old publisher layer list around, it should fail loudly.
        if k in ("layers", "basic_layers") and isinstance(v, list):
            assert "rgb" not in v, f"{path} publishes 'rgb' but semantic/rgb fusion is removed"
        if k == "data_type":
            assert v == "pointcloud", f"{path} sets data_type={v!r}; supported surface is pointcloud only"

    # Validate subscriber schema for any file that defines subscribers.
    for node_name, node_cfg in data.items():
        if not isinstance(node_cfg, dict):
            continue
        params = node_cfg.get("ros__parameters", {})
        if not isinstance(params, dict):
            continue
        subs = params.get("subscribers")
        if subs is None:
            continue
        assert isinstance(subs, dict), f"{path} subscribers must be a dict"
        for sub_name, sub_cfg in subs.items():
            assert isinstance(sub_cfg, dict), f"{path} subscriber '{sub_name}' must be a dict"
            assert sub_cfg.get("data_type") == "pointcloud", (
                f"{path} subscriber '{sub_name}' must be pointcloud"
            )
            assert sub_cfg.get("topic_name"), f"{path} subscriber '{sub_name}' missing topic_name"
