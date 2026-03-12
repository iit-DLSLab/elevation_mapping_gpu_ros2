ROS2 Jazzy Release
******************************************************************

Release Target
==================================================================

This page tracks the current ROS2/Jazzy release state for the ``ros2`` branch.

* Branch: ``ros2``
* Release version: ``v2.1.0``
* ROS2 maintainer: Lorenzo Terenzi
* Release surface:

  * ``elevation_mapping_cupy`` Python/CuPy node
  * ``semantic_sensor`` package
  * TurtleBot3 example launches
  * semantic image and semantic pointcloud example launches

What Changed Since ``v2.0.0``
==================================================================

* Restored the ROS2 semantic workflow and brought back the
  ``semantic_sensor`` package.
* Fixed the semantic image ingestion path in ``elevation_mapping_node.py``.
* Added ROS2 branch CI on the self-hosted NVIDIA runner using the
  ``moleworks_ros`` container.
* Revalidated the example launch surface instead of leaving semantic launches
  as stale files.
* Rewrote the user-facing documentation for ROS2 Jazzy.

Validation
==================================================================

The ``v2.1.0`` release validation was executed on ``starship`` inside the
GPU-enabled ``moleworks_ros`` container.

Executed checks:

* semantic image runtime smoke
* semantic pointcloud runtime smoke
* ``PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest sensor_processing/semantic_sensor/test -q``
* ``colcon test --packages-select elevation_mapping_cupy``

Observed result:

* 24 tests
* 0 errors
* 0 failures
* 0 skipped

Known Issues
==================================================================

* Some container images do not ship ``torchvision``. CI installs a matching
  wheel at runtime.
* The integration test still logs ``rclpy.shutdown already called`` during
  teardown, but the test passes.
* A real GPU is still required at runtime. A CUDA userspace image without a
  loaded NVIDIA host driver is not enough.

Release Checklist
==================================================================

The branch is ready to tag once the maintainer is satisfied with the current
state.

Before pushing the release tag:

* confirm the docs build on GitHub Actions
* confirm the self-hosted ROS2 CI still passes on ``ros2``
* create the Git tag for ``v2.1.0``
