#
# Copyright (c) 2022, Takahiro Miki. All rights reserved.
# Licensed under the MIT license. See LICENSE file in the project root for details.
#
import cupy as cp
from typing import List
import cupyx.scipy.ndimage as ndimage
import numpy as np
import cv2 as cv
import logging

_LOGGER = logging.getLogger(__name__)

from .plugin_manager import PluginBase


class Inpainting(PluginBase):
    """
    This class is used for inpainting, a process of reconstructing lost or deteriorated parts of images and videos.

    Args:
        cell_n (int): The number of cells. Default is 100.
        method (str): The inpainting method. Options are 'telea' or 'ns' (Navier-Stokes). Default is 'telea'.
        **kwargs (): Additional keyword arguments.
    """

    def __init__(self, cell_n: int = 100, method: str = "telea", **kwargs):
        super().__init__()
        if method == "telea":
            self.method = cv.INPAINT_TELEA
        elif method == "ns":  # Navier-Stokes
            self.method = cv.INPAINT_NS
        else:  # default method
            self.method = cv.INPAINT_TELEA

    def __call__(
        self,
        elevation_map: cp.ndarray,
        layer_names: List[str],
        plugin_layers: cp.ndarray,
        plugin_layer_names: List[str],
        *args,
    ) -> cp.ndarray:
        """

        Args:
            elevation_map (cupy._core.core.ndarray):
            layer_names (List[str]):
            plugin_layers (cupy._core.core.ndarray):
            plugin_layer_names (List[str]):
            *args ():

        Returns:
            cupy._core.core.ndarray:
        """
        valid_layer = elevation_map[2]
        mask_np = cp.asnumpy((valid_layer < 0.5).astype("uint8"))
        elevation = elevation_map[0]
        finite_elevation = cp.isfinite(elevation)
        valid_mask = cp.logical_and(valid_layer > 0.5, finite_elevation)

        if (mask_np < 1).any():
            if not cp.any(valid_mask):
                return elevation

            h_valid = elevation[valid_mask]
            h_max = float(cp.asnumpy(h_valid.max()))
            h_min = float(cp.asnumpy(h_valid.min()))
            denom = h_max - h_min
            if denom <= 1e-6:
                _LOGGER.warning(
                    "Inpainting detected near-flat terrain (h_min=%.3f, h_max=%.3f); broadcasting height.",
                    h_min,
                    h_max,
                )
                filled = cp.full(elevation.shape, h_max, dtype=cp.float32)
            else:
                # Replace NaNs with the minimum elevation value.
                safe_elevation = cp.where(finite_elevation, elevation, h_min)
                scaled = cp.asnumpy((safe_elevation - h_min) * 255.0 / denom).astype("uint8")
                dst = cv.inpaint(scaled, mask_np, 1, self.method)
                h_inpainted = dst.astype(np.float32) * denom / 255.0 + h_min
                filled = cp.asarray(h_inpainted, dtype=cp.float32)

            # Ensure already-valid cells mirror the authoritative elevation layer.
            filled = cp.where(valid_mask, elevation, filled)
            return filled.astype(cp.float64)
        else:
            return elevation_map[0]
