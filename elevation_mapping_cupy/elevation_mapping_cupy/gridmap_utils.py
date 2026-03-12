import math
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout as MAL
from std_msgs.msg import MultiArrayDimension as MAD


def encode_layer_to_multiarray(array: np.ndarray, layout: str = "gridmap_column") -> Float32MultiArray:
    """Encode a (rows, cols) array into a Float32MultiArray."""
    arr = np.asarray(array, dtype=np.float32)
    rows, cols = arr.shape
    msg = Float32MultiArray()
    msg.layout = MAL()

    if layout == "gridmap_column":
        msg.layout.dim.append(MAD(label="column_index", size=cols, stride=rows * cols))
        msg.layout.dim.append(MAD(label="row_index", size=rows, stride=rows))
        msg.data = arr.flatten(order="F").tolist()
        return msg

    if layout == "row_major":
        msg.layout.dim.append(MAD(label="row_index", size=rows, stride=rows * cols))
        msg.layout.dim.append(MAD(label="column_index", size=cols, stride=cols))
        msg.data = arr.flatten(order="C").tolist()
        return msg

    raise ValueError(f"Unknown layout '{layout}'")


def decode_multiarray_to_rows_cols(name: str, array_msg: Float32MultiArray) -> np.ndarray:
    """Decode Float32MultiArray into (rows, cols) row-major array."""
    data_np = np.asarray(array_msg.data, dtype=np.float32)
    dims = array_msg.layout.dim

    if len(dims) >= 2 and dims[0].label and dims[1].label:
        label0 = dims[0].label
        label1 = dims[1].label

        if label0 == "row_index" and label1 == "column_index":
            rows = dims[0].size or 1
            cols = dims[1].size or (len(data_np) // rows if rows else 0)
            if rows * cols != data_np.size:
                raise ValueError(f"Layer '{name}' has inconsistent layout metadata.")
            return data_np.reshape((rows, cols), order="C")

        if label0 == "column_index" and label1 == "row_index":
            cols = dims[0].size or 1
            rows = dims[1].size or (len(data_np) // cols if cols else 0)
            if rows * cols != data_np.size:
                raise ValueError(f"Layer '{name}' has inconsistent layout metadata.")
            return data_np.reshape((rows, cols), order="F")

    if dims:
        cols = dims[0].size or 1
        rows = dims[1].size if len(dims) > 1 else (len(data_np) // cols if cols else len(data_np))
    else:
        cols = int(math.sqrt(len(data_np))) if len(data_np) else 0
        rows = cols
    if rows * cols != data_np.size:
        raise ValueError(f"Layer '{name}' has inconsistent layout metadata.")
    return data_np.reshape((rows, cols), order="C")
