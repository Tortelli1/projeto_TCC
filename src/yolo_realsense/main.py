import cv2
from yolo_realsense.utils import initialize_realsense, get_frame
from yolo_realsense.inference import YOLODetector

def main():
    pipeline = initialize_realsense()
    detector = YOLODetector("models/yolov8n.pt")

    try:
        while True:
            color_image, depth_image = get_frame(pipeline)
            if color_image is None:
                continue

            results = detector.detect(color_image)
            annotated_frame = detector.draw_results(color_image, results)

            cv2.imshow("YOLOv8 + RealSense", annotated_frame)

            if cv2.waitKey(1) & 0xFF == 27:  # ESC para sair
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
