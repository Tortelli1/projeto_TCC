# src/yolo_realsense/inference.py
from ultralytics import YOLO

class YOLODetector:
    def __init__(self, model_path="models/yolov8n.pt", device='cpu', conf=0.25):
        self.model = YOLO(model_path)
        self.device = device
        self.conf = conf

    def detect(self, frame):
        # ultralytics aceita np.ndarray como input
        results = self.model(frame, device=self.device, conf=self.conf)
        return results

    def draw_results(self, results):
        # results[0].plot() retorna np.ndarray anotado
        return results[0].plot()