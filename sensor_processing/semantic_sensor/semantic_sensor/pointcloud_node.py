#!/usr/bin/env python3

import cupy as cp
import message_filters
import numpy as np
import rclpy
import ros2_numpy as rnp
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from semantic_sensor.config_io import load_sensor_config
from semantic_sensor.networks import resolve_model
from semantic_sensor.pointcloud_parameters import PointcloudParameter
from semantic_sensor.utils import decode_max


class SemanticPointcloudNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "semantic_pointcloud_node",
            automatically_declare_parameters_from_overrides=True,
        )

        sensor_name = self._require_string_parameter("sensor_name")
        config_path = self._optional_string_parameter("config_path")
        sensor_config = load_sensor_config(sensor_name, config_path or None)

        self.param = PointcloudParameter.from_dict(sensor_config)
        self.param.sensor_name = sensor_name
        self.param.feature_config.input_size = [80, 160]
        self.get_logger().info(f"Loaded semantic pointcloud config for '{sensor_name}'.")

        self.cv_bridge = CvBridge()
        self.P = None
        self.header = None
        self.prediction_img = None
        self.feat_img = None

        self.feature_extractor = None
        self.semantic_model = None
        self.segmentation_channels = None
        self.feature_channels = None

        self.create_custom_dtype()
        self.initialize_semantics()
        self.register_sub_pub()

    def _require_string_parameter(self, name: str) -> str:
        if not self.has_parameter(name):
            raise ValueError(f"Required parameter '{name}' was not provided.")
        value = self.get_parameter(name).get_parameter_value().string_value
        if not value:
            raise ValueError(f"Required parameter '{name}' must not be empty.")
        return value

    def _optional_string_parameter(self, name: str) -> str:
        if not self.has_parameter(name):
            return ""
        return self.get_parameter(name).get_parameter_value().string_value

    def initialize_semantics(self) -> None:
        if self.param.semantic_segmentation:
            self.semantic_model = resolve_model(self.param.segmentation_model, self.param)
            self.segmentation_channels = {}
            for channel, fusion in zip(self.param.channels, self.param.fusion):
                if fusion in ["class_bayesian", "class_average", "class_max"]:
                    self.segmentation_channels[channel] = fusion
            if not self.segmentation_channels:
                raise ValueError("semantic_segmentation is enabled but no semantic fusion channels were configured.")

        if self.param.feature_extractor:
            self.feature_extractor = resolve_model(self.param.feature_config.name, self.param.feature_config)
            self.feature_channels = {}
            for channel, fusion in zip(self.param.channels, self.param.fusion):
                if fusion in ["average"]:
                    self.feature_channels[channel] = fusion
            if not self.feature_channels:
                raise ValueError("feature_extractor is enabled but no feature channels were configured.")

    def register_sub_pub(self) -> None:
        self.create_subscription(CameraInfo, self.param.cam_info_topic, self.cam_info_callback, 2)

        rgb_sub = message_filters.Subscriber(self, Image, self.param.image_topic)
        depth_sub = message_filters.Subscriber(self, Image, self.param.depth_topic)
        subscribers = [depth_sub, rgb_sub]
        if self.param.confidence:
            confidence_sub = message_filters.Subscriber(self, Image, self.param.confidence_topic)
            subscribers.append(confidence_sub)

        self._sync = message_filters.ApproximateTimeSynchronizer(subscribers, queue_size=10, slop=0.5)
        self._sync.registerCallback(self.image_callback)
        self._subscribers = subscribers

        self.pcl_pub = self.create_publisher(PointCloud2, self.param.topic_name, 2)
        if self.param.publish_segmentation_image:
            self.seg_pub = self.create_publisher(Image, self.param.segmentation_image_topic, 2)
        if self.param.feature_extractor and self.param.publish_feature_image:
            self.feat_pub = self.create_publisher(Image, self.param.feature_config.feature_image_topic, 2)

    def create_custom_dtype(self) -> None:
        self.dtype = [("x", np.float32), ("y", np.float32), ("z", np.float32)]
        for channel in self.param.channels:
            self.dtype.append((channel, np.float32))

    def cam_info_callback(self, msg: CameraInfo) -> None:
        self.P = cp.asarray(msg.p, dtype=cp.float32).reshape(3, 4)
        self.height = msg.height
        self.width = msg.width
        self.header = msg.header

    def image_callback(self, depth_msg: Image, rgb_msg: Image = None, confidence_msg: Image = None) -> None:
        if self.P is None:
            return

        image = None
        confidence = None
        if rgb_msg is not None:
            image = cp.asarray(self.cv_bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8"))
        depth = cp.asarray(self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough"))
        if confidence_msg is not None:
            confidence = cp.asarray(self.cv_bridge.imgmsg_to_cv2(confidence_msg, desired_encoding="passthrough"))

        pcl = self.create_pcl_from_image(image, depth, confidence)
        self.publish_pointcloud(pcl, depth_msg.header)

        if self.param.publish_segmentation_image and self.param.semantic_segmentation and self.prediction_img is not None:
            self.publish_segmentation_image(self.prediction_img)
        if self.param.publish_feature_image and self.param.feature_extractor and self.feat_img is not None:
            self.publish_feature_image(self.feat_img)

    def create_pcl_from_image(self, image, depth, confidence):
        u, v = self.get_coordinates(depth, confidence)
        world_x = (u.astype(np.float32) - self.P[0, 2]) * depth[v, u] / self.P[0, 0]
        world_y = (v.astype(np.float32) - self.P[1, 2]) * depth[v, u] / self.P[1, 1]
        world_z = depth[v, u]

        points = np.zeros(world_x.shape, dtype=self.dtype)
        points["x"] = cp.asnumpy(world_x)
        points["y"] = cp.asnumpy(world_y)
        points["z"] = cp.asnumpy(world_z)
        self.process_image(image, u, v, points)
        return points

    def get_coordinates(self, depth, confidence):
        pos = cp.where(depth > 0, 1, 0)
        low = cp.where(depth < 8, 1, 0)
        conf = cp.where(confidence >= self.param.confidence_threshold, 1, 0) if confidence is not None else cp.ones(pos.shape)
        fin = cp.isfinite(depth)
        mask = cp.nonzero(cp.maximum(cp.rint(fin + pos + conf + low - 2.6), 0))
        u = mask[1]
        v = mask[0]
        return u, v

    def process_image(self, image, u, v, points) -> None:
        if image is not None and "color" in self.param.fusion:
            valid_rgb = image[v, u].get()
            r = np.asarray(valid_rgb[:, 0], dtype=np.uint32)
            g = np.asarray(valid_rgb[:, 1], dtype=np.uint32)
            b = np.asarray(valid_rgb[:, 2], dtype=np.uint32)
            rgb_arr = np.array((r << 16) | (g << 8) | b, dtype=np.uint32)
            rgb_arr.dtype = np.float32
            points[self.param.channels[self.param.fusion.index("color")]] = rgb_arr

        if image is not None and self.segmentation_channels is not None:
            self.perform_segmentation(image, points, u, v)
        if image is not None and self.feature_channels is not None:
            self.extract_features(image, points, u, v)

    def perform_segmentation(self, image, points, u, v) -> None:
        prediction = self.semantic_model["model"](image)
        values = prediction[:, v.get(), u.get()].get()
        for idx, channel in enumerate(self.semantic_model["model"].actual_channels):
            points[channel] = values[idx]
        if self.param.publish_segmentation_image:
            self.prediction_img = prediction

    def extract_features(self, image, points, u, v) -> None:
        prediction = self.feature_extractor["model"](image.get())
        values = prediction[:, v.get(), u.get()].cpu().detach().numpy()
        for idx, channel in enumerate(self.feature_channels.keys()):
            points[channel] = values[idx]
        if self.param.publish_feature_image:
            self.feat_img = prediction

    def publish_segmentation_image(self, probabilities) -> None:
        colors = cp.asarray(self.color_map(len(self.labels)))
        if "class_max" in self.param.fusion:
            prob = cp.zeros((len(self.labels),) + probabilities.shape[1:])
            prediction_idx = 0
            for channel, fusion in zip(self.param.channels, self.param.fusion):
                if fusion == "class_max":
                    temp = probabilities[prediction_idx]
                    temp_p, temp_i = decode_max(temp)
                    temp_i.choose(prob)
                    grid = cp.mgrid[0:temp_i.shape[0], 0:temp_i.shape[1]]
                    prob[temp_i, grid[0], grid[1]] = temp_p
                    prediction_idx += 1
                elif fusion in ["class_bayesian", "class_average"] and channel in self.semantic_model["model"].segmentation_channels:
                    prob[self.semantic_model["model"].segmentation_channels[channel]] = probabilities[prediction_idx]
                    prediction_idx += 1
            img = cp.argmax(prob, axis=0)
        else:
            img = cp.argmax(probabilities, axis=0)

        img = colors[img].astype(cp.uint8).get()
        seg_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding="rgb8")
        seg_msg.header = self.header
        self.seg_pub.publish(seg_msg)

    def publish_feature_image(self, features) -> None:
        from sklearn.decomposition import PCA

        data = np.reshape(features.cpu().detach().numpy(), (features.shape[0], -1)).T
        pca = PCA(n_components=3).fit(data)
        pca_descriptors = pca.transform(data)
        img_pca = pca_descriptors.reshape(features.shape[1], features.shape[2], 3)
        comp_min = img_pca.min(axis=(0, 1))
        comp_max = img_pca.max(axis=(0, 1))
        comp_img = (img_pca - comp_min) / (comp_max - comp_min)
        comp_img = (comp_img * 255).astype(np.uint8)
        feat_msg = self.cv_bridge.cv2_to_imgmsg(comp_img, encoding="passthrough")
        feat_msg.header = self.header
        self.feat_pub.publish(feat_msg)

    def publish_pointcloud(self, pcl, header) -> None:
        pc2 = rnp.msgify(PointCloud2, pcl)
        pc2.header = header
        self.pcl_pub.publish(pc2)

    def color_map(self, n: int = 256, normalized: bool = False) -> np.ndarray:
        def bitget(byteval: int, idx: int) -> bool:
            return (byteval & (1 << idx)) != 0

        dtype = "float32" if normalized else "uint8"
        cmap = np.zeros((n + 1, 3), dtype=dtype)
        for i in range(n + 1):
            r = g = b = 0
            c = i
            for j in range(8):
                r = r | (bitget(c, 0) << 7 - j)
                g = g | (bitget(c, 1) << 7 - j)
                b = b | (bitget(c, 2) << 7 - j)
                c = c >> 3
            cmap[i] = np.array([r, g, b])
        cmap[1] = np.array([188, 63, 59])
        cmap[2] = np.array([81, 113, 162])
        cmap[3] = np.array([136, 49, 132])
        return cmap[1:] / 255 if normalized else cmap[1:]

    @property
    def labels(self):
        if not self.param.semantic_segmentation:
            return []
        if "class_max" in self.param.fusion:
            return self.semantic_model["model"].get_classes()
        return list(self.segmentation_channels.keys())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticPointcloudNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
