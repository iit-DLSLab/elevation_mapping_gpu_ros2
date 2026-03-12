from pathlib import Path

import pytest

from semantic_sensor.config_io import load_sensor_config


def test_load_sensor_config_reads_restored_configs():
    repo_root = Path(__file__).resolve().parents[3]
    config_path = repo_root / "sensor_processing" / "semantic_sensor" / "config" / "sensor_parameter.yaml"

    image_cfg = load_sensor_config("front_cam_image", str(config_path))
    pointcloud_cfg = load_sensor_config("front_cam_pointcloud", str(config_path))

    assert image_cfg["publish_topic"] == "semantic_image"
    assert image_cfg["feature_topic"] == "semantic_seg_feat"
    assert pointcloud_cfg["topic_name"] == "semantic_pointcloud"


def test_load_sensor_config_requires_known_sensor_name():
    repo_root = Path(__file__).resolve().parents[3]
    config_path = repo_root / "sensor_processing" / "semantic_sensor" / "config" / "sensor_parameter.yaml"

    with pytest.raises(KeyError):
        load_sensor_config("missing_sensor", str(config_path))
