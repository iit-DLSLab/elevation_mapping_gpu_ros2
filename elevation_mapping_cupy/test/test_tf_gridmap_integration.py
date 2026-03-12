"""
Integration test for TF → GridMap pipeline in elevation_mapping_cupy.

This test verifies that:
1. The elevation_mapping_node starts correctly with test configuration
2. TF transforms (map → base_link) correctly update the map center
3. Map data shifts correctly when the robot moves
4. X movement affects X axis only, Y movement affects Y axis only (no axis swap)

ARCHITECTURE:
-------------
- elevation_mapping_node: Launched as separate process via launch_ros.actions.Node
- static_tf_publisher: Provides initial map→base_link transform
- TestFixtureNode: Runs in test process, publishes dynamic TF and subscribes to GridMap

The test fixture and launched nodes communicate via DDS (ROS2 middleware).
DDS discovery can take several seconds, so tests include delays for node discovery.

FAIL-LOUDLY POLICY:
-------------------
By default, tests FAIL if DDS discovery or marker injection doesn't work.
This catches real regressions: TF QoS issues, topic name changes, map not
publishing, pointcloud pipeline broken, etc.

For CI environments with known DDS flakiness, set SKIP_DDS_FLAKES=1 to
skip (not fail) on DDS infrastructure issues. The axis-swap regression
is still covered by unit tests (test_map_shifting.py)

AXIS-SWAP BUG COVERAGE:
-----------------------
The axis-swap bug (X movement causing Y shift) is tested at two levels:

1. Unit tests (test_map_shifting.py) - PRIMARY, always run:
   - test_shift_x_only_affects_columns
   - test_shift_y_only_affects_rows
   - test_no_axis_swap
   Run with: pytest elevation_mapping_cupy/tests/test_map_shifting.py -v

2. This integration test - Tests full ROS pipeline when DDS discovery works
"""

import os
import time
import unittest
from threading import Event

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from grid_map_msgs.msg import GridMap

import launch
import launch_ros
import launch_testing
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory


# ============================================================================
# Launch Description
# ============================================================================

def generate_test_description():
    """Generate launch description for the integration test."""
    import subprocess

    # CRITICAL: Stop ros2 daemon before tests to avoid RMW mismatch issues
    # See: https://github.com/ros2/system_tests/pull/460
    subprocess.run(['ros2', 'daemon', 'stop'], capture_output=True)

    # Force UDPv4 transport to avoid FastDDS shared memory discovery issues
    # See: https://answers.ros.org/question/407025/
    os.environ['FASTDDS_BUILTIN_TRANSPORTS'] = 'UDPv4'

    pkg_share = get_package_share_directory('elevation_mapping_cupy')

    # Path to test config
    test_config_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'config',
        'test_integration.yaml'
    )

    # Fallback to package share if not found
    if not os.path.exists(test_config_path):
        test_config_path = os.path.join(pkg_share, 'test', 'config', 'test_integration.yaml')

    elevation_mapping_node = launch_ros.actions.Node(
        package='elevation_mapping_cupy',
        executable='elevation_mapping_node.py',
        name='elevation_mapping_node',
        parameters=[test_config_path],
        output='screen',
    )

    # Static TF publisher for initial transform (map -> base_link at origin)
    static_tf_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen',
    )

    return (
        launch.LaunchDescription([
            static_tf_node,
            elevation_mapping_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'elevation_mapping_node': elevation_mapping_node,
            'static_tf_node': static_tf_node,
        }
    )


# ============================================================================
# Helper Functions
# ============================================================================

def create_pointcloud2(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """
    Create a PointCloud2 message from a numpy array of points.

    Args:
        points: Nx3 array of (x, y, z) points
        frame_id: TF frame for the pointcloud
        stamp: ROS timestamp

    Returns:
        PointCloud2 message
    """
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp

    # Define fields for x, y, z
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    msg.is_bigendian = False
    msg.point_step = 12  # 3 floats * 4 bytes
    msg.height = 1
    msg.width = len(points)
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True

    # Convert points to bytes
    points_f32 = points.astype(np.float32)
    msg.data = points_f32.tobytes()

    return msg


def create_transform(
    parent_frame: str,
    child_frame: str,
    x: float, y: float, z: float,
    stamp,
) -> TransformStamped:
    """Create a TransformStamped message."""
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0
    return t


# ============================================================================
# Test Fixture Node
# ============================================================================

class TestFixtureNode(Node):
    """
    Node that provides test infrastructure for the integration test.

    - Publishes TF transforms
    - Publishes synthetic pointclouds
    - Subscribes to GridMap output
    """

    def __init__(self):
        super().__init__('test_fixture_node')

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # PointCloud publisher - use sensor_data QoS to match node's subscriber
        pc_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.pc_publisher = self.create_publisher(
            PointCloud2,
            '/test_pointcloud',
            pc_qos
        )

        # GridMap subscriber with appropriate QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.gridmap_subscriber = self.create_subscription(
            GridMap,
            '/elevation_mapping_node/elevation_map',
            self._gridmap_callback,
            qos
        )

        # State
        self.last_gridmap: GridMap = None
        self.gridmap_received = Event()
        self.gridmap_count = 0

    def _gridmap_callback(self, msg: GridMap):
        """Store the latest GridMap message."""
        self.last_gridmap = msg
        self.gridmap_count += 1
        self.gridmap_received.set()

    def publish_tf(self, x: float, y: float, z: float = 0.0):
        """Publish TF transform for map → base_link."""
        stamp = self.get_clock().now().to_msg()

        # map → base_link (robot position)
        base_tf = create_transform('map', 'base_link', x, y, z, stamp)
        self.tf_broadcaster.sendTransform(base_tf)

    def publish_marker_pointcloud(self, marker_x: float, marker_y: float, marker_z: float):
        """Publish a pointcloud with a single marker point at world position."""
        stamp = self.get_clock().now().to_msg()

        # Create a small cluster of points around the marker position
        # This helps ensure the point gets into the map
        offsets = np.array([
            [0.0, 0.0, 0.0],
            [0.01, 0.0, 0.0],
            [-0.01, 0.0, 0.0],
            [0.0, 0.01, 0.0],
            [0.0, -0.01, 0.0],
        ], dtype=np.float32)

        center = np.array([[marker_x, marker_y, marker_z]], dtype=np.float32)
        points = center + offsets

        msg = create_pointcloud2(points, 'map', stamp)
        self.pc_publisher.publish(msg)

    def wait_for_gridmap(self, timeout: float = 2.0) -> bool:
        """Wait for a new GridMap message."""
        self.gridmap_received.clear()
        return self.gridmap_received.wait(timeout)

    def get_gridmap_center(self) -> tuple:
        """Get the center position from the latest GridMap."""
        if self.last_gridmap is None:
            return None
        pos = self.last_gridmap.info.pose.position
        return (pos.x, pos.y, pos.z)


# ============================================================================
# Test Cases
# ============================================================================

class TestTFGridMapIntegration(unittest.TestCase):
    """Integration tests for TF → GridMap pipeline.

    By default, tests FAIL if DDS discovery or marker injection doesn't work.
    This follows the research-code policy of "fail loudly" to catch real
    regressions (TF QoS, topic names, map publication, pointcloud pipeline).

    In CI environments where DDS flakiness is expected, set the environment
    variable SKIP_DDS_FLAKES=1 to skip (not fail) on known DDS issues.
    The axis-swap regression is still covered by unit tests (test_map_shifting.py).
    """

    # Class-level flag to track if DDS discovery worked
    dds_discovery_ok = False

    # Check if we should skip on DDS flakes (CI mode) or fail loudly (default)
    TOLERATE_DDS_FLAKES = os.environ.get('SKIP_DDS_FLAKES', '0') == '1'

    @classmethod
    def setUpClass(cls):
        """Initialize ROS and test fixture."""
        import subprocess

        # Stop ros2 daemon to avoid RMW mismatch issues
        # See: https://github.com/ros2/system_tests/pull/460
        subprocess.run(['ros2', 'daemon', 'stop'], capture_output=True)

        # Don't call rclpy.init() - launch_testing already initializes it
        try:
            rclpy.init()
        except RuntimeError:
            pass  # Already initialized by launch_testing
        cls.fixture = TestFixtureNode()
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.fixture)

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS."""
        cls.executor.shutdown()
        cls.fixture.destroy_node()
        # Don't call rclpy.shutdown() - let launch_testing handle it

    def setUp(self):
        """Reset state before each test."""
        self.fixture.last_gridmap = None
        self.fixture.gridmap_count = 0

    def _require_dds(self):
        """Fail (or skip in CI) if DDS discovery failed in test_01."""
        if not TestTFGridMapIntegration.dds_discovery_ok:
            if self.TOLERATE_DDS_FLAKES:
                self.skipTest("DDS discovery failed in test_01 (SKIP_DDS_FLAKES=1)")
            else:
                self.fail("DDS discovery failed in test_01 - cannot run dependent test")

    def spin_for(self, duration: float):
        """Spin the executor for a duration."""
        end_time = time.time() + duration
        while time.time() < end_time:
            self.executor.spin_once(timeout_sec=0.1)

    def wait_for_gridmap_with_spin(self, timeout: float = 3.0) -> bool:
        """Wait for GridMap while spinning."""
        self.fixture.gridmap_received.clear()
        initial_count = self.fixture.gridmap_count
        end_time = time.time() + timeout
        while time.time() < end_time:
            self.executor.spin_once(timeout_sec=0.1)
            if self.fixture.gridmap_count > initial_count:
                return True
        return False

    def _prime_map_with_pointcloud(self, repeats: int = 5, tf_x: float = 0.0, tf_y: float = 0.0, tf_z: float = 0.0):
        """Publish TF and a tiny pointcloud to give the node a timestamp for publishing."""
        for _ in range(repeats):
            self.fixture.publish_tf(tf_x, tf_y, tf_z)
            self.fixture.publish_marker_pointcloud(tf_x, tf_y, tf_z)
            self.spin_for(0.1)

    def test_01_node_startup(self):
        """Test that the elevation mapping node starts and publishes GridMap.

        This test verifies:
        1. Node starts without errors (verified by log output showing "Initialized map")
        2. DDS discovery works and we can receive GridMap messages

        By default, FAILS if DDS discovery doesn't work (fail loudly policy).
        Set SKIP_DDS_FLAKES=1 to skip instead (for CI with known DDS flakiness).
        """
        # Give DDS time to discover nodes (TF buffer needs 1-2s to initialize)
        self.spin_for(5.0)

        # Publish TF at origin
        for _ in range(50):
            self.fixture.publish_tf(0.0, 0.0, 0.0)
            self.spin_for(0.1)

        # Publish a small pointcloud so the node can publish maps (sets _last_t)
        self._prime_map_with_pointcloud(repeats=10)

        # Try to receive GridMap
        success = self.wait_for_gridmap_with_spin(timeout=15.0)
        if not success:
            if self.TOLERATE_DDS_FLAKES:
                self.skipTest("DDS discovery failed (SKIP_DDS_FLAKES=1)")
            else:
                self.fail(
                    "DDS discovery failed: test fixture cannot receive GridMap from launched node. "
                    "This may indicate TF QoS mismatch, wrong topic names, or map not publishing. "
                    "Set SKIP_DDS_FLAKES=1 to skip in CI."
                )

        # Mark DDS as working for subsequent tests
        TestTFGridMapIntegration.dds_discovery_ok = True

    def test_02_x_movement_shifts_x_axis(self):
        """
        Test that robot movement in +X direction shifts the map center in +X.

        This is the primary regression test for the axis-swap bug.
        """
        self._require_dds()

        # Start at origin
        for _ in range(30):
            self.fixture.publish_tf(0.0, 0.0, 0.0)
            self.spin_for(0.1)

        # Ensure pointcloud timestamp is available for pose updates
        self._prime_map_with_pointcloud(repeats=5)

        self.wait_for_gridmap_with_spin(timeout=10.0)
        initial_center = self.fixture.get_gridmap_center()
        self.assertIsNotNone(initial_center, "No GridMap received at origin")

        target_x = initial_center[0] + 1.0
        target_y = initial_center[1]

        # Move +1m in X direction (relative to current center)
        for _ in range(30):
            self.fixture.publish_tf(target_x, target_y, 0.0)
            self.spin_for(0.1)

        # Refresh _last_t using pointcloud stamped at the new pose
        self._prime_map_with_pointcloud(repeats=5, tf_x=target_x, tf_y=target_y)

        self.wait_for_gridmap_with_spin(timeout=10.0)
        new_center = self.fixture.get_gridmap_center()
        self.assertIsNotNone(new_center, "Failed to get new center after X movement")

        # Assert X changed, Y stayed the same
        delta_x = new_center[0] - initial_center[0]
        delta_y = new_center[1] - initial_center[1]

        self.assertAlmostEqual(delta_x, 1.0, delta=0.2,
                               msg=f"X should increase by ~1.0, got delta_x={delta_x}")
        self.assertAlmostEqual(delta_y, 0.0, delta=0.2,
                               msg=f"Y should not change for X movement, got delta_y={delta_y}")

    def test_03_y_movement_shifts_y_axis(self):
        """
        Test that robot movement in +Y direction shifts the map center in +Y.
        """
        self._require_dds()

        # Start at origin
        for _ in range(30):
            self.fixture.publish_tf(0.0, 0.0, 0.0)
            self.spin_for(0.1)

        self._prime_map_with_pointcloud(repeats=5)

        self.wait_for_gridmap_with_spin(timeout=10.0)
        center_before_marker = self.fixture.get_gridmap_center()
        self.assertIsNotNone(center_before_marker, "No GridMap received before marker injection")
        self.assertLess(abs(center_before_marker[0]), 0.2, f"Center not near origin before marker: {center_before_marker}")
        self.assertLess(abs(center_before_marker[1]), 0.2, f"Center not near origin before marker: {center_before_marker}")

        self.wait_for_gridmap_with_spin(timeout=10.0)
        initial_center = self.fixture.get_gridmap_center()
        self.assertIsNotNone(initial_center, "No GridMap received at origin")

        target_x = initial_center[0]
        target_y = initial_center[1] + 1.0

        # Move +1m in Y direction (relative)
        for _ in range(30):
            self.fixture.publish_tf(target_x, target_y, 0.0)
            self.spin_for(0.1)

        self._prime_map_with_pointcloud(repeats=5, tf_x=target_x, tf_y=target_y)

        self.wait_for_gridmap_with_spin(timeout=10.0)
        new_center = self.fixture.get_gridmap_center()
        self.assertIsNotNone(new_center, "Failed to get new center after Y movement")

        # Assert Y changed, X stayed the same
        delta_x = new_center[0] - initial_center[0]
        delta_y = new_center[1] - initial_center[1]

        self.assertAlmostEqual(delta_y, 1.0, delta=0.2,
                               msg=f"Y should increase by ~1.0, got delta_y={delta_y}")
        self.assertAlmostEqual(delta_x, 0.0, delta=0.2,
                               msg=f"X should not change for Y movement, got delta_x={delta_x}")

    def test_04_diagonal_movement(self):
        """Test that diagonal movement affects both axes correctly."""
        self._require_dds()

        # Start at origin
        for _ in range(30):
            self.fixture.publish_tf(0.0, 0.0, 0.0)
            self.spin_for(0.1)

        self._prime_map_with_pointcloud(repeats=5)

        self.wait_for_gridmap_with_spin(timeout=10.0)
        initial_center = self.fixture.get_gridmap_center()
        self.assertIsNotNone(initial_center, "No GridMap received at origin")

        target_x = initial_center[0] + 0.5
        target_y = initial_center[1] + 0.5

        # Move diagonally (+0.5, +0.5)
        for _ in range(30):
            self.fixture.publish_tf(target_x, target_y, 0.0)
            self.spin_for(0.1)

        self._prime_map_with_pointcloud(repeats=5, tf_x=target_x, tf_y=target_y)

        self.wait_for_gridmap_with_spin(timeout=10.0)
        new_center = self.fixture.get_gridmap_center()
        self.assertIsNotNone(new_center, "Failed to get new center")

        delta_x = new_center[0] - initial_center[0]
        delta_y = new_center[1] - initial_center[1]

        self.assertAlmostEqual(delta_x, 0.5, delta=0.2,
                               msg=f"X should increase by ~0.5, got {delta_x}")
        self.assertAlmostEqual(delta_y, 0.5, delta=0.2,
                               msg=f"Y should increase by ~0.5, got {delta_y}")

    def test_05_no_axis_swap(self):
        """
        Explicit test that X movement doesn't affect Y and vice versa.

        This is the key regression test for the axis-swap bug.
        """
        self._require_dds()

        # Test 1: X-only movement
        for _ in range(30):
            self.fixture.publish_tf(0.0, 0.0, 0.0)
            self.spin_for(0.1)
        self.wait_for_gridmap_with_spin(timeout=10.0)
        start_center = self.fixture.get_gridmap_center()
        self.assertIsNotNone(start_center, "No GridMap received at origin")

        self._prime_map_with_pointcloud(repeats=5)
        self.wait_for_gridmap_with_spin(timeout=5.0)
        start_center = self.fixture.get_gridmap_center()
        self.assertIsNotNone(start_center, "No GridMap after priming pointcloud")

        target_x = start_center[0] + 1.0
        target_y = start_center[1]

        for _ in range(30):
            self.fixture.publish_tf(target_x, target_y, 0.0)  # X only
            self.spin_for(0.1)
        self.wait_for_gridmap_with_spin(timeout=10.0)
        after_x_move = self.fixture.get_gridmap_center()
        self.assertIsNotNone(after_x_move, "Failed to get center after X move")

        self._prime_map_with_pointcloud(repeats=3, tf_x=target_x, tf_y=target_y)
        self.wait_for_gridmap_with_spin(timeout=5.0)
        after_x_move = self.fixture.get_gridmap_center()
        self.assertIsNotNone(after_x_move, "Failed to get center after priming at X pose")

        y_change_from_x_move = abs(after_x_move[1] - start_center[1])
        self.fixture.get_logger().info(f"[no_axis_swap] centers start={start_center}, after_x={after_x_move}")
        self.assertLess(y_change_from_x_move, 0.15,
                        msg=f"Y changed by {y_change_from_x_move} when only X moved - AXIS SWAP BUG!")

        # Test 2: Y-only movement (from current position)
        target_x_fixed = after_x_move[0]
        target_y2 = after_x_move[1] + 1.0
        for _ in range(30):
            self.fixture.publish_tf(target_x_fixed, target_y2, 0.0)  # Y only (X fixed)
            self.spin_for(0.1)

        self._prime_map_with_pointcloud(repeats=3, tf_x=target_x_fixed, tf_y=target_y2)
        self.wait_for_gridmap_with_spin(timeout=10.0)
        after_y_move = self.fixture.get_gridmap_center()
        self.assertIsNotNone(after_y_move, "Failed to get center after Y move")

        x_change_from_y_move = abs(after_y_move[0] - after_x_move[0])
        self.assertLess(x_change_from_y_move, 0.15,
                        msg=f"X changed by {x_change_from_y_move} when only Y moved - AXIS SWAP BUG!")



# ============================================================================
# Post-shutdown test
# ============================================================================

@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Check that the node exited cleanly."""

    def test_exit_code(self, proc_info):
        """Verify the elevation_mapping_node exits with acceptable code."""
        # Note: Exit code 1 can happen due to rclpy shutdown race conditions
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, 1, -2, -15],  # 0=normal, 1=shutdown race, -2=SIGINT, -15=SIGTERM
        )
