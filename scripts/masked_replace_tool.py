#!/usr/bin/env python3
"""
Utility for issuing masked_replace requests to elevation_mapping_cupy.
Useful to test the masked_replace service.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from math import ceil
from typing import Dict, Optional

import numpy as np

import rclpy
from rclpy.node import Node

from grid_map_msgs.msg import GridMap
from grid_map_msgs.srv import SetGridMap
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension


def positive_float(value: str) -> float:
    val = float(value)
    if val <= 0.0:
        raise argparse.ArgumentTypeError("Value must be > 0.")
    return val


def non_negative_float(value: str) -> float:
    val = float(value)
    if val < 0.0:
        raise argparse.ArgumentTypeError("Value must be >= 0.")
    return val


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Send a masked_replace patch to elevation_mapping_cupy.")
    parser.add_argument("--service", default="/elevation_mapping_cupy/masked_replace", help="Service name to call.")
    parser.add_argument("--mask-layer", default="mask", help="Name of the mask layer expected by the node.")
    parser.add_argument("--frame", default="odom", help="Frame ID used in the GridMap header.")
    parser.add_argument("--center-x", type=float, default=0.0, help="Patch center X coordinate (meters).")
    parser.add_argument("--center-y", type=float, default=0.0, help="Patch center Y coordinate (meters).")
    parser.add_argument("--center-z", type=float, default=0.0, help="Patch center Z coordinate (meters).")
    parser.add_argument("--size-x", type=positive_float, default=1.0, help="Patch length in X (meters).")
    parser.add_argument("--size-y", type=positive_float, default=1.0, help="Patch length in Y (meters).")
    parser.add_argument(
        "--full-length-x",
        type=positive_float,
        default=None,
        help="Optional total GridMap length in X (meters). If set, a full-size map is sent and only the patch region is marked in the mask."
    )
    parser.add_argument(
        "--full-length-y",
        type=positive_float,
        default=None,
        help="Optional total GridMap length in Y (meters). If set, a full-size map is sent and only the patch region is marked in the mask."
    )
    parser.add_argument(
        "--full-center-x",
        type=float,
        default=0.0,
        help="GridMap center X (meters) to use when sending a full-size map. Defaults to 0."
    )
    parser.add_argument(
        "--full-center-y",
        type=float,
        default=0.0,
        help="GridMap center Y (meters) to use when sending a full-size map. Defaults to 0."
    )
    parser.add_argument("--resolution", type=positive_float, default=0.1, help="Grid resolution (meters per cell).")
    parser.add_argument("--elevation", type=float, default=0.1, help="Elevation value to set (meters).")
    parser.add_argument("--variance", type=non_negative_float, default=0.05, help="Variance value to set.")
    parser.add_argument("--mask-value", type=float, default=1.0, help="Mask value applied over the patch (NaN keeps cells untouched).")
    parser.add_argument("--valid-layer", dest="valid_layer", action="store_true", help="Also set the 'is_valid' layer to 1.0 in the patch region (default).")
    parser.add_argument("--no-valid-layer", dest="valid_layer", action="store_false", help="Do not modify the 'is_valid' layer.")
    parser.add_argument(
        "--invalidate-first",
        dest="invalidate_first",
        action="store_true",
        help="Emulate the LiDAR update flow by first invalidating cells (set is_valid=0) before writing new values.",
    )
    parser.add_argument(
        "--no-invalidate-first",
        dest="invalidate_first",
        action="store_false",
        help="Write the new data directly without a preceding invalidation pass.",
    )
    parser.set_defaults(valid_layer=True)
    parser.set_defaults(invalidate_first=False)
    return parser


@dataclass
class PatchConfig:
    center_x: float
    center_y: float
    center_z: float
    length_x: float
    length_y: float
    resolution: float
    frame_id: str
    mask_layer: str
    elevation: float
    variance: float
    mask_value: float
    add_valid_layer: bool
    invalidate_first: bool
    full_length_x: Optional[float] = None
    full_length_y: Optional[float] = None
    full_center_x: float = 0.0
    full_center_y: float = 0.0

    @property
    def shape(self) -> Dict[str, int]:
        cols = max(1, ceil(self.length_x / self.resolution))
        rows = max(1, ceil(self.length_y / self.resolution))
        return {"rows": rows, "cols": cols}

    @property
    def actual_length_x(self) -> float:
        return self.shape["cols"] * self.resolution

    @property
    def actual_length_y(self) -> float:
        return self.shape["rows"] * self.resolution


class MaskedReplaceClient(Node):
    def __init__(self, service_name: str, config: PatchConfig):
        super().__init__("masked_replace_client")
        self._config = config
        self._client = self.create_client(SetGridMap, service_name)
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for service {service_name} ...")

    def send_request(self):
        cfg = self._config
        if cfg.invalidate_first and not cfg.add_valid_layer:
            self.get_logger().warning(
                "--invalidate-first requested but --no-valid-layer is set; falling back to single-pass update."
            )
        if cfg.invalidate_first and cfg.add_valid_layer:
            self._call_stage("invalidate", self._build_validity_message(value=0.0))
            self._call_stage("update-while-invalid", self._build_data_message(valid_value=None))
            self._call_stage("revalidate", self._build_validity_message(value=1.0))
        else:
            valid_value = 1.0 if cfg.add_valid_layer else None
            self._call_stage("update", self._build_data_message(valid_value=valid_value))

    def _call_stage(self, label: str, grid_map: GridMap):
        self.get_logger().info(f"Issuing masked_replace stage '{label}'.")
        req = SetGridMap.Request()
        req.map = grid_map
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("masked_replace request completed.")
        else:
            self.get_logger().error(f"masked_replace call failed: {future.exception()}")

    def _base_grid_map(self) -> GridMap:
        cfg = self._config
        gm = GridMap()
        gm.header.frame_id = cfg.frame_id
        gm.header.stamp = self.get_clock().now().to_msg()
        gm.info.resolution = cfg.resolution
        # If full map was requested, use the full lengths and center the GridMap at the full-map center.
        if cfg.full_length_x or cfg.full_length_y:
            gm.info.length_x = cfg.full_length_x or cfg.actual_length_x
            gm.info.length_y = cfg.full_length_y or cfg.actual_length_y
            gm.info.pose.position.x = cfg.full_center_x
            gm.info.pose.position.y = cfg.full_center_y
        else:
            gm.info.length_x = cfg.actual_length_x
            gm.info.length_y = cfg.actual_length_y
            gm.info.pose.position.x = cfg.center_x
            gm.info.pose.position.y = cfg.center_y
        gm.info.pose.position.z = cfg.center_z
        gm.info.pose.orientation.w = 1.0
        gm.basic_layers = ["elevation"]
        return gm

    def _mask_array(self, force_value: Optional[float] = None) -> np.ndarray:
        cfg = self._config
        rows = cfg.shape["rows"]
        cols = cfg.shape["cols"]
        mask_value = cfg.mask_value if force_value is None else force_value
        if np.isnan(mask_value):
            mask_value = 1.0
        return np.full((rows, cols), mask_value, dtype=np.float32)

    def _make_full_arrays(self) -> Dict[str, np.ndarray]:
        """Create full-size arrays (possibly larger than the patch) and place the patch in them."""
        cfg = self._config
        length_x = cfg.full_length_x or cfg.length_x
        length_y = cfg.full_length_y or cfg.length_y
        cols_full = max(1, ceil(length_x / cfg.resolution))
        rows_full = max(1, ceil(length_y / cfg.resolution))

        # Base arrays filled with NaN (masked out)
        mask_full = np.full((rows_full, cols_full), np.nan, dtype=np.float32)
        elev_full = np.full((rows_full, cols_full), np.nan, dtype=np.float32)
        var_full = np.full((rows_full, cols_full), np.nan, dtype=np.float32)
        valid_full = np.zeros((rows_full, cols_full), dtype=np.float32)

        # Patch dimensions and offset within the full map
        patch_rows = cfg.shape["rows"]
        patch_cols = cfg.shape["cols"]
        row_offset = int(round(cfg.center_y / cfg.resolution))
        col_offset = int(round(cfg.center_x / cfg.resolution))
        row_start = rows_full // 2 + row_offset - patch_rows // 2
        col_start = cols_full // 2 + col_offset - patch_cols // 2
        row_end = row_start + patch_rows
        col_end = col_start + patch_cols

        # Clamp if window would exceed bounds
        if row_start < 0 or col_start < 0 or row_end > rows_full or col_end > cols_full:
            raise ValueError("Patch exceeds full map bounds; adjust center/size or full map length.")

        mask_val = cfg.mask_value
        if np.isnan(mask_val):
            mask_val = 1.0
        mask_full[row_start:row_end, col_start:col_end] = mask_val
        elev_full[row_start:row_end, col_start:col_end] = cfg.elevation
        var_full[row_start:row_end, col_start:col_end] = cfg.variance
        if cfg.add_valid_layer:
            valid_full[row_start:row_end, col_start:col_end] = 1.0

        return {
            "mask": mask_full,
            "elevation": elev_full,
            "variance": var_full,
            "is_valid": valid_full,
            "rows_full": rows_full,
            "cols_full": cols_full,
        }

    def _build_validity_message(self, value: float) -> GridMap:
        gm = self._base_grid_map()
        cfg = self._config
        if cfg.full_length_x or cfg.full_length_y:
            arrays_full = self._make_full_arrays()
            rows_full = arrays_full["rows_full"]
            cols_full = arrays_full["cols_full"]
            gm.layers = [cfg.mask_layer, "is_valid"]
            arrays = {
                cfg.mask_layer: arrays_full["mask"],
                "is_valid": np.full((rows_full, cols_full), value, dtype=np.float32),
            }
        else:
            mask = self._mask_array()
            rows, cols = mask.shape
            gm.layers = [cfg.mask_layer, "is_valid"]
            arrays = {
                cfg.mask_layer: mask,
                "is_valid": np.full((rows, cols), value, dtype=np.float32),
            }
        for layer in gm.layers:
            gm.data.append(self._numpy_to_multiarray(arrays[layer]))
        return gm

    def _build_data_message(self, valid_value: Optional[float]) -> GridMap:
        gm = self._base_grid_map()
        cfg = self._config
        if cfg.full_length_x or cfg.full_length_y:
            arrays_full = self._make_full_arrays()
            gm.layers = [cfg.mask_layer, "elevation", "variance"]
            arrays = {
                cfg.mask_layer: arrays_full["mask"],
                "elevation": arrays_full["elevation"],
                "variance": arrays_full["variance"],
            }
            if valid_value is not None:
                gm.layers.append("is_valid")
                arrays["is_valid"] = arrays_full["is_valid"]
        else:
            mask = self._mask_array()
            rows, cols = mask.shape
            gm.layers = [cfg.mask_layer, "elevation", "variance"]
            arrays = {
                cfg.mask_layer: mask,
                "elevation": np.full((rows, cols), cfg.elevation, dtype=np.float32),
                "variance": np.full((rows, cols), cfg.variance, dtype=np.float32),
            }
            if valid_value is not None:
                gm.layers.append("is_valid")
                arrays["is_valid"] = np.full((rows, cols), valid_value, dtype=np.float32)
        for layer in gm.layers:
            gm.data.append(self._numpy_to_multiarray(arrays[layer]))
        return gm

    @staticmethod
    def _numpy_to_multiarray(array: np.ndarray) -> Float32MultiArray:
        msg = Float32MultiArray()
        layout = MultiArrayLayout()
        rows, cols = array.shape
        # numpy
        layout.dim.append(MultiArrayDimension(label="column_index", size=cols, stride=rows * cols))
        layout.dim.append(MultiArrayDimension(label="row_index", size=rows, stride=rows))
        msg.layout = layout
        msg.data = array.flatten(order="F").tolist()
        return msg


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    cfg = PatchConfig(
        center_x=args.center_x,
        center_y=args.center_y,
        center_z=args.center_z,
        length_x=args.size_x,
        length_y=args.size_y,
        resolution=args.resolution,
        frame_id=args.frame,
        mask_layer=args.mask_layer,
        elevation=args.elevation,
        variance=args.variance,
        mask_value=args.mask_value,
        add_valid_layer=args.valid_layer,
        invalidate_first=args.invalidate_first,
        full_length_x=args.full_length_x,
        full_length_y=args.full_length_y,
        full_center_x=args.full_center_x,
        full_center_y=args.full_center_y,
    )

    rclpy.init()
    node = MaskedReplaceClient(args.service, cfg)
    node.send_request()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
