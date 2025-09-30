from ultralytics import YOLO
import cv2

class YOLODetector:
    def __init__(self, model_path="models/yolov8n.pt"):
        self.model = YOLO(model_path)

    def detect(self, frame):
        results = self.model(frame)
        return results

    def draw_results(self, frame, results):
        annotated_frame = results[0].plot()
        return annotated_frame
