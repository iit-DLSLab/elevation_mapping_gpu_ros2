from __future__ import annotations

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def default_config_path() -> Path:
    return Path(get_package_share_directory("semantic_sensor")) / "config" / "sensor_parameter.yaml"


def load_sensor_config(sensor_name: str, config_path: str | None = None) -> dict:
    if not sensor_name:
        raise ValueError("sensor_name must be provided.")

    path = Path(config_path).expanduser() if config_path else default_config_path()
    if not path.exists():
        raise FileNotFoundError(f"Semantic sensor config not found: {path}")

    data = yaml.safe_load(path.read_text()) or {}
    if sensor_name not in data:
        raise KeyError(f"Sensor '{sensor_name}' not found in config '{path}'.")

    sensor_cfg = data[sensor_name]
    if not isinstance(sensor_cfg, dict):
        raise TypeError(f"Sensor config for '{sensor_name}' must be a mapping.")
    return sensor_cfg
