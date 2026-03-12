"""
Integration test: save_map + load_map services.

This launches the elevation_mapping_node with a tiny deterministic config,
publishes TF + a synthetic pointcloud, then verifies that:
  1) /elevation_mapping_cupy/save_map succeeds and writes bags
  2) /elevation_mapping_cupy/load_map succeeds and publishing continues

Fail-loudly policy:
  - No automatic skipping on missing CUDA/GPU.
  - DDS discovery issues fail by default.
"""

import os
import tempfile
import time
import unittest
from threading import Event

import numpy as np

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from grid_map_msgs.msg import GridMap
from grid_map_msgs.srv import ProcessFile

import launch
import launch_ros
import launch_testing
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory


def _create_pointcloud2(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.height = 1
    msg.width = len(points)
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = points.astype(np.float32).tobytes()
    return msg


def _create_transform(parent: str, child: str, x: float, y: float, z: float, stamp) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = float(x)
    t.transform.translation.y = float(y)
    t.transform.translation.z = float(z)
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0
    return t


def generate_test_description():
    import subprocess

    subprocess.run(["ros2", "daemon", "stop"], capture_output=True)
    os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"

    pkg_share = get_package_share_directory("elevation_mapping_cupy")
    test_config_path = os.path.join(pkg_share, "test", "config", "test_integration.yaml")
    if not os.path.exists(test_config_path):
        raise FileNotFoundError(f"Missing test config: {test_config_path}")

    elevation_mapping_node = launch_ros.actions.Node(
        package="elevation_mapping_cupy",
        executable="elevation_mapping_node.py",
        name="elevation_mapping_node",
        parameters=[test_config_path],
        output="screen",
    )

    static_tf_node = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"],
        output="screen",
    )

    return (
        launch.LaunchDescription(
            [
                static_tf_node,
                elevation_mapping_node,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "elevation_mapping_node": elevation_mapping_node,
        },
    )


class _Fixture(Node):
    def __init__(self):
        super().__init__("save_load_fixture")

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        pc_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.pc_pub = self.create_publisher(PointCloud2, "/test_pointcloud", pc_qos)

        gm_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.gridmap_evt = Event()
        self.last_gridmap: GridMap | None = None
        self.gridmap_sub = self.create_subscription(
            GridMap, "/elevation_mapping_node/elevation_map", self._on_gridmap, gm_qos
        )

        self.save_cli = self.create_client(ProcessFile, "/elevation_mapping_cupy/save_map")
        self.load_cli = self.create_client(ProcessFile, "/elevation_mapping_cupy/load_map")

    def _on_gridmap(self, msg: GridMap):
        self.last_gridmap = msg
        self.gridmap_evt.set()

    def publish_tf(self, x: float, y: float, z: float = 0.0):
        stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(_create_transform("map", "base_link", x, y, z, stamp))

    def publish_pointcloud_plane(self, z: float = 0.0):
        stamp = self.get_clock().now().to_msg()
        xs = np.linspace(0.5, 3.0, 40, dtype=np.float32)
        ys = np.linspace(-1.0, 1.0, 40, dtype=np.float32)
        X, Y = np.meshgrid(xs, ys, indexing="xy")
        Z = np.full_like(X, z, dtype=np.float32)
        pts = np.stack([X.reshape(-1), Y.reshape(-1), Z.reshape(-1)], axis=1)
        self.pc_pub.publish(_create_pointcloud2(pts, frame_id="base_link", stamp=stamp))


class TestSaveLoadServices(unittest.TestCase):
    def test_save_then_load(self):
        rclpy.init()
        node = _Fixture()
        exec_ = SingleThreadedExecutor()
        exec_.add_node(node)

        try:
            # Wait for services.
            t_end = time.time() + 15.0
            while time.time() < t_end and not node.save_cli.wait_for_service(timeout_sec=0.2):
                exec_.spin_once(timeout_sec=0.1)
            self.assertTrue(node.save_cli.service_is_ready(), "save_map service not available")

            t_end = time.time() + 15.0
            while time.time() < t_end and not node.load_cli.wait_for_service(timeout_sec=0.2):
                exec_.spin_once(timeout_sec=0.1)
            self.assertTrue(node.load_cli.service_is_ready(), "load_map service not available")

            # Publish some data for a short time.
            for k in range(30):
                node.publish_tf(0.02 * k, 0.0)
                node.publish_pointcloud_plane(z=0.0)
                exec_.spin_once(timeout_sec=0.1)

            # Ensure at least one gridmap message arrived.
            t_end = time.time() + 10.0
            while time.time() < t_end and not node.gridmap_evt.is_set():
                exec_.spin_once(timeout_sec=0.1)
            self.assertTrue(node.gridmap_evt.is_set(), "No GridMap received before save")
            node.gridmap_evt.clear()

            with tempfile.TemporaryDirectory() as td:
                fused_path = os.path.join(td, "emap")

                req = ProcessFile.Request()
                req.file_path = fused_path
                req.topic_name = ""

                fut = node.save_cli.call_async(req)
                t_end = time.time() + 30.0
                while time.time() < t_end and rclpy.ok() and not fut.done():
                    exec_.spin_once(timeout_sec=0.1)
                self.assertIsNotNone(fut.result(), "save_map returned no result")
                self.assertTrue(fut.result().success, "save_map failed")

                self.assertTrue(os.path.exists(fused_path), "Fused bag path not created")
                self.assertTrue(os.path.exists(fused_path + "_raw"), "Raw bag path not created")

                fut = node.load_cli.call_async(req)
                t_end = time.time() + 30.0
                while time.time() < t_end and rclpy.ok() and not fut.done():
                    exec_.spin_once(timeout_sec=0.1)
                self.assertIsNotNone(fut.result(), "load_map returned no result")
                self.assertTrue(fut.result().success, "load_map failed")

            # Publishing should continue after load.
            t_end = time.time() + 10.0
            got = False
            while time.time() < t_end:
                exec_.spin_once(timeout_sec=0.2)
                if node.gridmap_evt.is_set():
                    got = True
                    break
            self.assertTrue(got, "No GridMap received after load")

            # Basic sanity: at least some finite elevation values exist.
            gm = node.last_gridmap
            self.assertIsNotNone(gm)
            self.assertGreater(len(gm.data), 0)
            arr = np.array(gm.data[0].data, dtype=np.float32)
            self.assertTrue(np.isfinite(arr).any())

        finally:
            exec_.shutdown()
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, 1, -2, -15],
        )
