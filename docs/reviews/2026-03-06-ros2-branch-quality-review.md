# ROS2 Branch Quality Review (2026-03-06)

This note captures the review of the current `ros2` branch state. The checkout reviewed was clean and matched `origin/ros2`, so the findings below describe the branch as it stands rather than an unmerged diff.

## Main Findings

### Critical

- The image ingestion path is broken.
  - `elevation_mapping_cupy/scripts/elevation_mapping_node.py`
  - `elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping.py`
  - `image_callback()` calls `ElevationMap.input_image()` with the wrong argument order and without the expected image metadata contract. Any configured image subscriber should fail on first callback.

### High

- Launch files and parameter-file roots are inconsistent.
  - `elevation_mapping_cupy/launch/elevation_mapping_cupy.launch.py`
  - `elevation_mapping_cupy/launch/anymal.launch.py`
  - `elevation_mapping_cupy/config/core/core_param.yaml`
  - `elevation_mapping_cupy/config/setups/anymal/anymal_parameters.yaml`
  - `elevation_mapping_cupy/config/setups/anymal/anymal_sensor_parameter.yaml`
  - The launch files use node name `elevation_mapping`, while the YAML roots target `elevation_mapping_node` or `/elevation_mapping_node`. Parameter overrides will not bind as intended.

- Documentation is still largely ROS1/catkin-era and materially misleading for the ROS2 Jazzy branch.
  - `docs/source/getting_started/installation.rst`
  - `docs/source/getting_started/tutorial.rst`
  - `docs/source/getting_started/introduction.rst`

- Sphinx autodoc config is stale or broken.
  - `docs/source/conf.py`
  - The module search path points to a non-existent directory, and `autodoc_mock_imports` contains a malformed entry caused by a missing comma.

- The test/CI surface is not trustworthy enough.
  - `elevation_mapping_cupy/CMakeLists.txt`
  - `.github/workflows/python-tests.yml`
  - Only a small subset of unit tests runs in `colcon test`.
  - The workflow still uses Python 3.8 and `cupy-cuda11x`, while the branch docs/package metadata claim Python 3.10+ and CUDA 12.x.
  - The workflow still references a deleted `sensor_processing` test path.

- Image subscriber config parsing does not match the shipped YAML schema.
  - `elevation_mapping_cupy/scripts/elevation_mapping_node.py`
  - `elevation_mapping_cupy/config/core/example_setup.yaml`
  - `elevation_mapping_cupy/config/setups/anymal/anymal_sensor_parameter.yaml`
  - The node looks for `topic_name_camera` and `topic_name_camera_info`, while configs use `topic_name` and `camera_info_topic_name`.

- The ANYmal image config is incompatible with the implemented subscriber type.
  - `elevation_mapping_cupy/config/setups/anymal/anymal_sensor_parameter.yaml`
  - The topics are `/compressed`, but the node subscribes with `sensor_msgs/Image`, not `CompressedImage`.

### Medium

- `sensor_msgs_py` is imported but not declared as a package dependency.
  - `elevation_mapping_cupy/scripts/elevation_mapping_node.py`
  - `elevation_mapping_cupy/package.xml`

- The startup initializer is exposed in config but not actually wired through the node.
  - `elevation_mapping_cupy/scripts/elevation_mapping_node.py`
  - `elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping.py`

- The setuptools entry point is broken.
  - `elevation_mapping_cupy/setup.py`
  - It points at `elevation_mapping_cupy.elevation_mapping_node:main`, but the real executable lives in `scripts/elevation_mapping_node.py`.

- `turtlesim_init.launch.py` fails silently.
  - `elevation_mapping_cupy/launch/turtlesim_init.launch.py`
  - It catches package lookup errors and returns an empty launch description instead of failing loudly.

- Packaging/install logic is duplicated across CMake and setuptools.
  - `elevation_mapping_cupy/CMakeLists.txt`
  - `elevation_mapping_cupy/setup.py`

## Repository Hygiene

These files should not stay as-is on the branch:

- `TODO.md`
  - Completed checklist only.
- `index_refacto_plan.md`
  - Temporary planning scratchpad already describing itself as removable once the refactor is stable.
- `docs/layout_alignment_plan.md`
  - Internal layout memo, not maintained docs. Keep only if rewritten into a short permanent design note.
- `elevation_mapping_cupy/README.md`
  - Stale duplicate documentation with outdated launch arguments.

## Quality Improvement Plan

1. Fix runtime breakages first.
   - Repair `image_callback()` to call `input_image()` correctly.
   - Align image subscriber config keys with the YAML schema.
   - Decide whether image subscriptions support `sensor_msgs/Image`, `CompressedImage`, or both, then make configs and code match.

2. Fix launch and parameter binding.
   - Use one node name consistently across launch files and parameter YAML roots.
   - Add a launch-level regression test that verifies key parameters are actually loaded.

3. Remove silent failure paths.
   - Replace blanket `try/except: pass` parameter loading with explicit handling.
   - Make broken launches fail loudly.

4. Clean repository hygiene.
   - Delete `TODO.md`.
   - Delete `index_refacto_plan.md`.
   - Either delete `docs/layout_alignment_plan.md` or rewrite it into a short permanent conventions note.
   - Remove or merge `elevation_mapping_cupy/README.md`.

5. Rewrite the user-facing docs for ROS2 Jazzy.
   - Installation
   - Tutorial
   - Introduction
   - Testing page with `colcon test`, direct `pytest`, and `launch_testing` commands

6. Make CI honest.
   - Drop dead workflow steps.
   - Align Python/CUDA versions with the supported matrix.
   - Decide which tests are required in `colcon test` and register them explicitly.

7. Clean packaging.
   - Declare missing runtime deps.
   - Remove duplicate install paths.
   - Fix broken setuptools metadata and entry points.

## Validation Limits During Review

- `python3 -m pytest -q` failed at collection in this environment because `cupy` was not installed.
- ROS2 launch/integration tests were not runnable here because `/opt/ros/jazzy/setup.bash` was not present.
