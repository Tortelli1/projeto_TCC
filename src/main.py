import os
import cv2
from tqdm import tqdm
from ultralytics import YOLO
from inference import process_image
from utils import save_results, save_json

# --- Configurações ---
MODEL_PATH = os.path.join("models", "best.pt")
IMG_FOLDER = os.path.join("data", "test")
OUTPUT_FOLDER = os.path.join("results", "images")

TARGET_CLASSES = ['traffic cone', 'Boxes']

# --- Carregar modelo ---
model = YOLO(MODEL_PATH)

# --- Processar imagens ---
image_files = [f for f in os.listdir(IMG_FOLDER) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
detections_data = {}

for img_file in tqdm(image_files, desc="Processando imagens"):
    img_path = os.path.join(IMG_FOLDER, img_file)
    image = cv2.imread(img_path)

    if image is None:
        print(f"Erro ao carregar {img_path}")
        continue

    detections, annotated = process_image(model, image, TARGET_CLASSES)

    save_results(annotated, detections, OUTPUT_FOLDER, img_file)
    detections_data[img_file] = detections

# --- Salvar todas as detecções em JSON ---
save_json(detections_data, "results")

print("✅ Processamento concluído!")
