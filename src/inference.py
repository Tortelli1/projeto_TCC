from ultralytics import YOLO
import cv2

def process_image(model, image, target_classes):
    """
    Roda YOLO em uma imagem e retorna as detecções + imagem anotada.
    """
    results = model(image, verbose=False)
    detections = []

    for result in results:
        for box in result.boxes:
            cls_id = int(box.cls[0])
            class_name = model.names[cls_id]
            conf = float(box.conf[0])

            if class_name in target_classes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                detections.append({
                    'class': class_name,
                    'confidence': conf,
                    'bbox': [x1, y1, x2, y2]
                })

    annotated = results[0].plot()
    return detections, annotated