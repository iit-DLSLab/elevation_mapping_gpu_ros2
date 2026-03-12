import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension

import elevation_mapping_cupy.gridmap_utils as emn


def _make_manual_gridmap_column(rows: int, cols: int) -> tuple[np.ndarray, Float32MultiArray]:
    """Build a Float32MultiArray with GridMap-style column-major layout."""
    data = np.arange(rows * cols, dtype=np.float32).reshape((rows, cols))
    msg = Float32MultiArray()
    msg.layout = MultiArrayLayout()
    msg.layout.dim.append(MultiArrayDimension(label="column_index", size=cols, stride=rows * cols))
    msg.layout.dim.append(MultiArrayDimension(label="row_index", size=rows, stride=rows))
    msg.data = data.flatten(order="F").tolist()
    return data, msg


def test_encode_decode_column_major_roundtrip():
    arr = np.arange(12, dtype=np.float32).reshape((3, 4))
    msg = emn.encode_layer_to_multiarray(arr, layout="gridmap_column")
    out = emn.decode_multiarray_to_rows_cols("elevation", msg)
    assert np.array_equal(arr, out)


def test_encode_decode_row_major_roundtrip():
    arr = np.arange(6, dtype=np.float32).reshape((2, 3))
    msg = emn.encode_layer_to_multiarray(arr, layout="row_major")
    out = emn.decode_multiarray_to_rows_cols("elevation", msg)
    assert np.array_equal(arr, out)


def test_decode_manual_gridmap_column_major():
    arr, msg = _make_manual_gridmap_column(rows=3, cols=4)
    out = emn.decode_multiarray_to_rows_cols("elevation", msg)
    assert np.array_equal(arr, out)
