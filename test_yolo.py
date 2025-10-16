from ultralytics import YOLO
import torch

model = YOLO('models/yolov8n.pt')
print("Modelo carregado!")

print("CUDA disponível: ", torch.cuda.is_available())
if torch.cuda.is_available():
    print("GPU Detectada: ", torch.cuda.get_device_name(0))