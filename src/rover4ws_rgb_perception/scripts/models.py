"""Neural network wrappers: SegFormer (free-space segmentation) and YOLO (object detection)."""
import numpy as np

# ADE20K classes a ground robot may drive on. Matched against the first name in each
# label string (e.g. "earth, ground" -> "earth").
DRIVABLE_NAMES = {'floor', 'road', 'grass', 'sidewalk', 'earth', 'field', 'sand', 'path',
                  'dirt track', 'runway', 'land', 'rug'}
SKY_NAMES = {'sky'}

# COCO classes treated as (potentially) moving objects; every detection is an obstacle.
DYNAMIC_NAMES = {'person', 'bicycle', 'car', 'motorcycle', 'bus', 'truck', 'dog', 'cat',
                 'horse', 'sheep', 'cow', 'bird', 'skateboard', 'suitcase'}
# approximate footprint radius (m) per class for planning
CLASS_RADIUS = {'person': 0.3, 'bicycle': 0.6, 'car': 1.2, 'motorcycle': 0.7, 'bus': 2.5,
                'truck': 2.0, 'dog': 0.35, 'cat': 0.2}


class SegFormer:
    """ADE20K SegFormer -> per-pixel drivable / sky masks."""

    def __init__(self, model_name, input_hw, logger):
        import torch
        from transformers import AutoModelForSemanticSegmentation
        self.torch = torch
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.dtype = torch.float16 if self.device == 'cuda' else torch.float32
        logger.info(f'Loading {model_name} on {self.device} ...')
        self.model = AutoModelForSemanticSegmentation.from_pretrained(model_name).to(
            self.device, self.dtype).eval()
        self.input_hw = tuple(input_hw)
        labels = self.model.config.id2label
        first = {int(i): n.split(',')[0].strip() for i, n in labels.items()}
        self.labels = first
        n = len(first)
        self.drivable_lut = np.zeros(n, bool)
        self.sky_lut = np.zeros(n, bool)
        for i, name in first.items():
            self.drivable_lut[i] = name in DRIVABLE_NAMES
            self.sky_lut[i] = name in SKY_NAMES
        self.mean = torch.tensor([0.485, 0.456, 0.406], device=self.device).view(1, 3, 1, 1)
        self.std = torch.tensor([0.229, 0.224, 0.225], device=self.device).view(1, 3, 1, 1)

    def __call__(self, rgb, out_hw):
        """rgb HxWx3 uint8 -> label map at out_hw (int16)."""
        torch = self.torch
        with torch.no_grad():
            x = torch.from_numpy(rgb).to(self.device).permute(2, 0, 1)[None].float() / 255.0
            x = torch.nn.functional.interpolate(x, size=self.input_hw, mode='bilinear',
                                                align_corners=False)
            x = ((x - self.mean) / self.std).to(self.dtype)
            logits = self.model(pixel_values=x).logits
            logits = torch.nn.functional.interpolate(logits.float(), size=out_hw,
                                                     mode='bilinear', align_corners=False)
            return logits.argmax(1)[0].to(torch.int16).cpu().numpy()


class Yolo:
    """Ultralytics YOLO -> list of detections."""

    def __init__(self, weights, conf, logger):
        from ultralytics import YOLO
        logger.info(f'Loading YOLO {weights} ...')
        self.model = YOLO(weights)
        self.conf = conf
        self.names = self.model.names

    def __call__(self, bgr):
        r = self.model(bgr, verbose=False, conf=self.conf)[0]
        out = []
        for b, c, s in zip(r.boxes.xyxy.cpu().numpy(), r.boxes.cls.cpu().numpy(),
                           r.boxes.conf.cpu().numpy()):
            name = self.names[int(c)]
            out.append({'cls': name, 'conf': float(s), 'box': b.astype(float),
                        'dynamic': name in DYNAMIC_NAMES,
                        'radius': CLASS_RADIUS.get(name, 0.3)})
        return out
