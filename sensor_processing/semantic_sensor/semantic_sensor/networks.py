import cupy as cp
import numpy as np
import torch
import torch.nn.functional as NF
import torchvision.transforms.functional as TF
from torchvision.models.segmentation import FCN_ResNet50_Weights, fcn_resnet50
from torchvision.models.segmentation import (
    LRASPP_MobileNet_V3_Large_Weights,
    lraspp_mobilenet_v3_large,
)
from torchvision.models.segmentation import (
    DeepLabV3_MobileNet_V3_Large_Weights,
    deeplabv3_mobilenet_v3_large,
)
from torchvision.transforms import Resize

from semantic_sensor.pointcloud_parameters import FeatureExtractorParameter


def resolve_model(name, config=None):
    if name == "fcn_resnet50":
        return {
            "name": name,
            "model": PytorchModel(fcn_resnet50, FCN_ResNet50_Weights.DEFAULT, config),
        }
    if name == "lraspp_mobilenet_v3_large":
        return {
            "name": name,
            "model": PytorchModel(
                lraspp_mobilenet_v3_large,
                LRASPP_MobileNet_V3_Large_Weights.DEFAULT,
                config,
            ),
        }
    if name == "deeplabv3_mobilenet_v3_large":
        return {
            "name": name,
            "model": PytorchModel(
                deeplabv3_mobilenet_v3_large,
                DeepLabV3_MobileNet_V3_Large_Weights.COCO_WITH_VOC_LABELS_V1,
                config,
            ),
        }
    if name == "detectron_coco_panoptic_fpn_R_101_3x":
        try:
            from detectron2 import model_zoo
            from detectron2.config import get_cfg
            from detectron2.data import MetadataCatalog
            from detectron2.engine import DefaultPredictor
            from detectron2.utils.logger import setup_logger
        except ImportError as exc:
            raise ImportError(
                "detectron2 is required for detectron_coco_panoptic_fpn_R_101_3x."
            ) from exc

        setup_logger()
        return {
            "name": name,
            "model": DetectronModel(
                "COCO-PanopticSegmentation/panoptic_fpn_R_101_3x.yaml",
                config,
                model_zoo,
                get_cfg,
                MetadataCatalog,
                DefaultPredictor,
            ),
        }
    if name == "DINO":
        return {
            "name": config.model + str(config.patch_size),
            "model": STEGOModel(config.model + str(config.patch_size), config),
        }
    raise NotImplementedError(f"Unknown semantic sensor model '{name}'.")


def _semantic_channels(param) -> list[str]:
    if hasattr(param, "fusion"):
        return [
            channel
            for channel, fusion in zip(param.channels, param.fusion)
            if fusion in {"class_average", "class_bayesian"}
        ]
    return list(param.channels)


def _cupy_or_numpy_to_torch(image, device: torch.device) -> torch.Tensor:
    if isinstance(image, cp.ndarray):
        if device.type == "cuda":
            tensor = torch.as_tensor(image, device=device)
        else:
            tensor = torch.from_numpy(cp.asnumpy(image)).to(device)
    else:
        tensor = torch.as_tensor(image, device=device)
    return tensor


class PytorchModel:
    def __init__(self, net, weights, param):
        self.model = net(weights=weights)
        self.weights = weights
        self.param = param
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model.eval()
        self.model.to(device=self.device)
        self.resolve_categories()

    def resolve_categories(self) -> None:
        class_to_idx = {cls: idx for idx, cls in enumerate(self.get_classes())}
        requested = _semantic_channels(self.param)
        missing = [channel for channel in requested if channel not in class_to_idx]
        if missing:
            raise ValueError(
                f"Semantic classes {missing} are not available in model categories {self.get_classes()}."
            )

        self.segmentation_channels = {channel: class_to_idx[channel] for channel in requested}
        self.actual_channels = requested

    def __call__(self, image, *args, **kwargs):
        batch = _cupy_or_numpy_to_torch(image, self.device).permute(2, 0, 1).unsqueeze(0)
        batch = TF.convert_image_dtype(batch, torch.float32)
        with torch.no_grad():
            prediction = self.model(batch)["out"]
            normalized_masks = torch.squeeze(prediction.softmax(dim=1), dim=0)
            selected = normalized_masks[list(self.segmentation_channels.values())]
        return cp.asarray(selected)

    def get_classes(self):
        return self.weights.meta["categories"]


class DetectronModel:
    def __init__(self, weights, param, model_zoo, get_cfg, metadata_catalog, predictor_cls):
        self.param = param
        self.cfg = get_cfg()
        self.cfg.merge_from_file(model_zoo.get_config_file(weights))
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5
        self.cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(weights)
        self.predictor = predictor_cls(self.cfg)
        self.metadata = metadata_catalog.get(self.cfg.DATASETS.TRAIN[0])

        self.stuff_categories = self._resolve_categories(self.get_category("stuff_classes"))
        self.thing_categories = self._resolve_categories(self.get_category("thing_classes"))
        self.segmentation_channels = {}
        self.segmentation_channels.update(self.stuff_categories)
        self.segmentation_channels.update(self.thing_categories)
        self.actual_channels = list(self.segmentation_channels.keys())

    def _resolve_categories(self, categories: list[str]) -> dict[str, int]:
        class_to_idx = {cls: idx for idx, cls in enumerate(categories)}
        selected = {}
        for channel in _semantic_channels(self.param):
            if channel in class_to_idx:
                selected[channel] = class_to_idx[channel]
        return selected

    def __call__(self, image, *args, **kwargs):
        image_np = cp.asnumpy(cp.flip(image, axis=2))
        prediction = self.predictor(image_np)
        probabilities = cp.asarray(torch.softmax(prediction["sem_seg"], dim=0))
        output = cp.zeros(
            (len(self.segmentation_channels), probabilities.shape[1], probabilities.shape[2]),
            dtype=cp.float32,
        )

        for output_idx, channel in enumerate(self.actual_channels):
            if channel in self.stuff_categories:
                output[output_idx] = probabilities[self.stuff_categories[channel]]

        panoptic_indices, instance_info = prediction["panoptic_seg"]
        panoptic_indices = cp.asarray(panoptic_indices)
        for output_idx, channel in enumerate(self.actual_channels):
            if channel not in self.thing_categories:
                continue
            category_id = self.thing_categories[channel]
            for instance in instance_info:
                if instance is None or not instance["isthing"]:
                    continue
                if instance["category_id"] != category_id:
                    continue
                mask = (panoptic_indices == instance["id"]).astype(cp.float32)
                output[output_idx] = cp.maximum(output[output_idx], mask * instance["score"])

        return output

    def get_category(self, name):
        return self.metadata.get(name)

    def get_classes(self):
        return self.get_category("thing_classes") + self.get_category("stuff_classes")


class STEGOModel:
    def __init__(self, weights, cfg):
        try:
            from semantic_sensor.DINO.modules import DinoFeaturizer
        except ImportError as exc:
            raise ImportError("DINO feature extraction dependencies are missing.") from exc

        self.cfg: FeatureExtractorParameter = cfg
        self.model = DinoFeaturizer(weights, cfg=self.cfg)
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model.to(self.device)
        self.model.eval()
        self.shrink = Resize(
            size=(self.cfg.input_size[0], self.cfg.input_size[1]), antialias=True
        )

    def to_tensor(self, data):
        if isinstance(data, cp.ndarray):
            data = cp.asnumpy(data)
        data = data.astype(np.float32)
        if len(data.shape) == 3:
            data = data.transpose(2, 0, 1)
        elif len(data.shape) == 2:
            data = data.reshape((1,) + data.shape)
        if len(data.shape) == 3 and data.shape[0] == 3:
            data = data / 255.0
        return torch.as_tensor(data, device=self.device)

    def __call__(self, image, *args, **kwargs):
        image = self.to_tensor(image).unsqueeze(0)
        output_size = image.shape[-2:]
        image = self.shrink(image)
        image = TF.normalize(image, mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225))

        _, code1 = self.model(image)
        _, code2 = self.model(image.flip(dims=[3]))
        code = (code1 + code2.flip(dims=[3])) / 2
        code = NF.interpolate(code, output_size, mode=self.cfg.interpolation, align_corners=False).detach()
        return torch.squeeze(code, dim=0)
