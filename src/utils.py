import os
import json
import cv2

def save_results(image, detections, output_folder, img_file):
    """
    Salva imagem anotada e atualiza dicionário de detecções.
    """
    os.makedirs(output_folder, exist_ok=True)
    output_path = os.path.join(output_folder, f"resultado_{img_file}")
    cv2.imwrite(output_path, image)
    return detections

def save_json(data, output_folder, filename="detections.json"):
    """
    Salva dicionário de detecções em JSON.
    """
    with open(os.path.join(output_folder, filename), "w") as f:
        json.dump(data, f, indent=4)