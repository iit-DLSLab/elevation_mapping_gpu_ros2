#!/usr/bin/env bash

set -euo pipefail

TORCHVISION_VERSION=${TORCHVISION_VERSION:-0.25.0}
PYTORCH_EXTRA_INDEX_URL=${PYTORCH_EXTRA_INDEX_URL:-https://download.pytorch.org/whl/cu128}

echo "=== ROS environment ==="
set +u
source /opt/ros/jazzy/setup.bash
set -u

echo "=== GPU check ==="
python3 - <<'PY'
import cupy as cp

device_count = cp.cuda.runtime.getDeviceCount()
if device_count < 1:
    raise RuntimeError("No CUDA devices available to CuPy inside CI container.")
print(f"CuPy devices: {device_count}")
print(cp.cuda.runtime.getDeviceProperties(0)["name"].decode())
PY

echo "=== Torch / torchvision setup ==="
python3 -c 'import torch; print(torch.__version__)'
python3 -m pip install --no-input \
  "torchvision==${TORCHVISION_VERSION}" \
  --extra-index-url "${PYTORCH_EXTRA_INDEX_URL}"
python3 -c 'import torch, torchvision; print(torch.__version__); print(torchvision.__version__)'

echo "=== Build ==="
colcon build \
  --symlink-install \
  --packages-up-to semantic_sensor elevation_mapping_cupy \
  --cmake-args -DBUILD_TESTING=ON

echo "=== Workspace overlay ==="
set +u
source install/setup.bash
set -u

echo "=== Runtime smoke ==="
bash src/elevation_mapping_cupy/scripts/ci/runtime_smoke.sh

echo "=== semantic_sensor pytest ==="
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/elevation_mapping_cupy/sensor_processing/semantic_sensor/test \
  -q

echo "=== elevation_mapping_cupy colcon test ==="
colcon test \
  --packages-select elevation_mapping_cupy \
  --event-handlers console_direct+

echo "=== elevation_mapping_cupy test results ==="
colcon test-result \
  --test-result-base build/elevation_mapping_cupy \
  --verbose
