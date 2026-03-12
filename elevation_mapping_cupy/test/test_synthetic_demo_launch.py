"""
Smoke-test the *actual* golden-path launch file:

  ros2 launch elevation_mapping_cupy synthetic_depth_demo.launch.py launch_rviz:=false

This validates that:
  - the synthetic TF + PointCloud2 publisher starts
  - elevation_mapping_node starts with the shipped config files
  - GridMap output is published on the expected topic

We intentionally keep assertions minimal here; deeper behavior is covered by:
  - unit tests (kernel + map shifting)
  - integration tests (TF-driven shifting + save/load services)
"""

import os
import time
import unittest
from threading import Event

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from grid_map_msgs.msg import GridMap

import launch
import launch_testing
import launch_testing.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_test_description():
    import subprocess

    subprocess.run(["ros2", "daemon", "stop"], capture_output=True)
    os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"

    pkg_share = get_package_share_directory("elevation_mapping_cupy")
    launch_path = os.path.join(pkg_share, "launch", "synthetic_depth_demo.launch.py")
    if not os.path.exists(launch_path):
        raise FileNotFoundError(f"Missing launch file: {launch_path}")

    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={
            "launch_rviz": "false",
        }.items(),
    )

    return (
        launch.LaunchDescription(
            [
                demo,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class _GridMapWaiter(Node):
    def __init__(self):
        super().__init__("synthetic_demo_waiter")
        self._evt = Event()
        self.last_msg: GridMap | None = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self._sub = self.create_subscription(
            GridMap,
            "/elevation_mapping_node/elevation_map",
            self._on_msg,
            qos,
        )

    def _on_msg(self, msg: GridMap):
        self.last_msg = msg
        self._evt.set()

    def wait_for_msg(self, timeout_s: float) -> bool:
        t_end = time.time() + timeout_s
        exec_ = SingleThreadedExecutor()
        exec_.add_node(self)
        try:
            while time.time() < t_end and rclpy.ok():
                if self._evt.is_set():
                    return True
                exec_.spin_once(timeout_sec=0.2)
            return self._evt.is_set()
        finally:
            exec_.shutdown()


class TestSyntheticDemoLaunch(unittest.TestCase):
    def test_gridmap_is_published(self):
        rclpy.init()
        node = _GridMapWaiter()
        try:
            ok = node.wait_for_msg(timeout_s=20.0)
            self.assertTrue(ok, "Timed out waiting for /elevation_mapping_node/elevation_map")
            msg = node.last_msg
            self.assertIsNotNone(msg)
            self.assertEqual(msg.header.frame_id, "map")
            self.assertGreater(msg.info.length_x, 0.0)
            self.assertGreater(msg.info.length_y, 0.0)
            self.assertGreater(len(msg.layers), 0)
            self.assertEqual(len(msg.layers), len(msg.data))
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, 1, -2, -15],
        )

