"""ROS-independent YOLO result handling for traffic signals."""
from __future__ import annotations

from typing import Iterable, List, NamedTuple, Optional, Sequence


TRAFFIC_CLASSES = {
    'red': 'RED',
    'yellow': 'YELLOW',
    'green': 'GREEN',
    'left_arrow': 'LEFT_ARROW',
}
LANE_CLASSES = {
    'down_arrow': 'DOWN',
    'x_sign': 'X',
}


class Detection(NamedTuple):
    class_name: str
    confidence: float
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    @property
    def center_x(self) -> float:
        return (self.x_min + self.x_max) * 0.5

class FrameDecision(NamedTuple):
    signal: str
    signal_confidence: float
    lane_left: str
    lane_right: str
    lane_confidence: float


def _best(items: Iterable[Detection]) -> Optional[Detection]:
    return max(items, key=lambda item: item.confidence, default=None)


def decide_frame(detections: Sequence[Detection], image_width: int) -> FrameDecision:
    """Convert detections into the traffic message consumed by State Manager.

    The highest-confidence traffic class wins. DOWN/X signs are assigned to
    the left or right finish branch by bounding-box centre. Missing detections
    remain UNKNOWN; absence is never interpreted as permission.
    """
    traffic = _best(item for item in detections if item.class_name in TRAFFIC_CLASSES)
    if traffic is None:
        signal, signal_confidence = 'UNKNOWN', 0.0
    else:
        signal = TRAFFIC_CLASSES[traffic.class_name]
        signal_confidence = traffic.confidence

    lane_candidates = [item for item in detections if item.class_name in LANE_CLASSES]
    split_x = max(0, image_width) * 0.5
    left = _best(item for item in lane_candidates if item.center_x < split_x)
    right = _best(item for item in lane_candidates if item.center_x >= split_x)
    lane_left = LANE_CLASSES[left.class_name] if left else 'UNKNOWN'
    lane_right = LANE_CLASSES[right.class_name] if right else 'UNKNOWN'
    observed = [item.confidence for item in (left, right) if item]
    lane_confidence = min(observed) if observed else 0.0

    return FrameDecision(signal, signal_confidence,
                         lane_left, lane_right, lane_confidence)


class TrafficLightDetector:
    """Small adapter around an Ultralytics YOLO model.

    The model object is injected so the decision logic can be tested without
    importing Torch or Ultralytics.
    """

    def __init__(self, model, confidence: float = 0.5,
                 image_size: int = 640, device: str = ''):
        if not 0.0 < confidence <= 1.0:
            raise ValueError('confidence must be in (0, 1]')
        if image_size <= 0:
            raise ValueError('image_size must be positive')
        self.model = model
        self.confidence = float(confidence)
        self.image_size = int(image_size)
        self.device = str(device).strip()

    def detect(self, frame) -> List[Detection]:
        options = {
            'conf': self.confidence,
            'imgsz': self.image_size,
            'verbose': False,
        }
        if self.device and self.device.lower() != 'auto':
            options['device'] = self.device
        results = self.model.predict(frame, **options)
        names = self.model.names
        detections = []
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                class_name = names[class_id] if isinstance(names, dict) else names[class_id]
                coordinates = box.xyxy[0]
                if hasattr(coordinates, 'tolist'):
                    coordinates = coordinates.tolist()
                detections.append(Detection(
                    str(class_name), float(box.conf[0]),
                    float(coordinates[0]), float(coordinates[1]),
                    float(coordinates[2]), float(coordinates[3]),
                ))
        return detections
