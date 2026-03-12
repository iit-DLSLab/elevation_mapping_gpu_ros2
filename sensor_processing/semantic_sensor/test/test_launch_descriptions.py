import importlib.util
from pathlib import Path

from launch import LaunchDescription


def load_launch(path: Path) -> LaunchDescription:
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_semantic_sensor_launch_descriptions_generate():
    repo_root = Path(__file__).resolve().parents[3]
    launch_files = [
        repo_root / "sensor_processing" / "semantic_sensor" / "launch" / "semantic_image.launch.py",
        repo_root / "sensor_processing" / "semantic_sensor" / "launch" / "semantic_pointcloud.launch.py",
    ]

    for launch_file in launch_files:
        description = load_launch(launch_file)
        assert isinstance(description, LaunchDescription)
        assert description.entities
