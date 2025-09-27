"""
Nó ROS2 para detecção em tempo real usando YOLOv8 e Intel RealSense.
Assina imagens coloridas e de profundidade, roda detecção, calcula distância e publica resultados.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import message_filters
import numpy as np
import json
import os
import cv2
from ultralytics import YOLO

class YoloRealtimeDetector(Node):
    def __init__(self):
        super().__init__('yolo_realsense_detector')

        # Parâmetros configuráveis via arquivo YAML ou CLI
        self.declare_parameter('model_path', os.path.join('models', 'yolov8n.pt'))
        self.declare_parameter('target_classes', [])
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('publish_image_topic', '/yolo/image')
        self.declare_parameter('publish_detections_topic', '/yolo/detections')
        self.declare_parameter('conf_threshold', 0.25)

        # Carrega parâmetros
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.target_classes = self.get_parameter('target_classes').get_parameter_value().string_array_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.pub_image_topic = self.get_parameter('publish_image_topic').get_parameter_value().string_value
        self.pub_dets_topic = self.get_parameter('publish_detections_topic').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value

        self.get_logger().info(f'Carregando modelo YOLO: {model_path} (device={self.device})')
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f'Falha ao carregar modelo YOLO: {e}')
            raise

        self.bridge = CvBridge()

        # Subscribers sincronizados
        color_sub = message_filters.Subscriber(self, Image, color_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        ats = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=10, slop=0.1)
        ats.registerCallback(self.callback_rgb_depth)

        # Publishers
        self.pub_image = self.create_publisher(Image, self.pub_image_topic, 10)
        self.pub_detections = self.create_publisher(String, self.pub_dets_topic, 10)

        self.get_logger().info('YOLO RealSense detector inicializado.')

    def _depth_to_meters(self, depth_image):
        """
        Converte imagem de profundidade para metros.
        """
        if depth_image.dtype == np.uint16:
            return depth_image.astype(np.float32) / 1000.0
        elif depth_image.dtype in [np.float32, np.float64]:
            return depth_image.astype(np.float32)
        else:
            self.get_logger().warn('Tipo de profundidade desconhecido, convertendo para float32.')
            return depth_image.astype(np.float32)

    def _filter_classes(self, class_name):
        """
        Filtra classes conforme parâmetro target_classes.
        """
        return not self.target_classes or class_name in self.target_classes

    def callback_rgb_depth(self, color_msg, depth_msg):
        try:
            color_cv = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_cv = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Falha na conversão CvBridge: {e}')
            return

        depth_m = self._depth_to_meters(depth_cv)

        try:
            results = self.model.predict(
                source=color_cv,
                device=self.device,
                verbose=False,
                conf=self.conf_threshold,
                imgsz=640
            )
        except Exception as e:
            self.get_logger().error(f'Falha na inferência YOLO: {e}')
            return

        res = results[0]
        dets_out = []

        for box in res.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            class_name = self.model.names[cls_id]

            if not self._filter_classes(class_name):
                continue

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            h, w = depth_m.shape[:2]
            if cx < 0 or cy < 0 or cx >= w or cy >= h:
                distance_m = float('nan')
            else:
                distance_m = float(depth_m[cy, cx])

            dets_out.append({
                'class': class_name,
                'confidence': conf,
                'bbox': [x1, y1, x2, y2],
                'center': [cx, cy],
                'distance_m': None if np.isnan(distance_m) else round(distance_m, 3)
            })

            label = f'{class_name} {conf:.2f} {"" if np.isnan(distance_m) else f"{distance_m:.2f}m"}'
            cv2.rectangle(color_cv, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(color_cv, label, (x1, max(y1 - 8, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publica imagem anotada
        try:
            out_img_msg = self.bridge.cv2_to_imgmsg(color_cv, encoding='bgr8')
            out_img_msg.header = color_msg.header
            self.pub_image.publish(out_img_msg)
        except Exception as e:
            self.get_logger().error(f'Falha ao publicar imagem anotada: {e}')

        # Publica detections como JSON
        try:
            payload = {
                'header': {
                    'stamp': {
                        'sec': color_msg.header.stamp.sec,
                        'nanosec': color_msg.header.stamp.nanosec
                    },
                    'frame_id': color_msg.header.frame_id
                },
                'detections': dets_out
            }
            msg = String()
            msg.data = json.dumps(payload)
            self.pub_detections.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Falha ao publicar JSON de detecções: {e}')

def main(args=None):
    """
    Função principal do nó ROS2.
    """
    rclpy.init(args=args)
    node = YoloRealtimeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()