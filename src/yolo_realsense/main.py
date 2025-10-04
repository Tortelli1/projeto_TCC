# src/yolo_realsense/main.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from yolo_realsense.inference import YOLODetector
from yolo_realsense.utils import RealSenseDirect, cv2_to_msg
import numpy as np

class YOLORealsenseNode(Node):
    def __init__(self):
        super().__init__('yolo_realsense_node')

        # parâmetros (com defaults)
        self.declare_parameter('use_driver_node', True)
        self.declare_parameter('model_path', 'models/yolov8n.pt')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('camera_topic', '/camera/color/image_raw')

        self.use_driver_node = self.get_parameter('use_driver_node').get_parameter_value().bool_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        device = self.get_parameter('device').get_parameter_value().string_value
        conf = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.publish_annotated = self.get_parameter('publish_annotated').get_parameter_value().bool_value
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        self.get_logger().info(f'use_driver_node={self.use_driver_node}, model={model_path}, device={device}, conf={conf}')

        # detector
        self.detector = YOLODetector(model_path=model_path, device=device, conf=conf)
        self.bridge = CvBridge()

        # publisher para imagem anotada
        if self.publish_annotated:
            self.pub_image = self.create_publisher(Image, 'yolo/annotated_image', 10)

        # se for usar driver realsense2_camera, subcreve no tópico de imagem
        if self.use_driver_node:
            from sensor_msgs.msg import Image as SensorImage
            self.create_subscription(SensorImage, camera_topic, self.image_callback, 10)
            self.get_logger().info(f'Subscribed to camera topic: {camera_topic}')
        else:
            # usa RealSense direto via pyrealsense2
            try:
                self.realsense = RealSenseDirect()
            except Exception as e:
                self.get_logger().error(f'Falha ao inicializar RealSense: {e}')
                raise

            # timer para capturar frames
            self.create_timer(1.0 / 30.0, self.timer_callback)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        # detect
        results = self.detector.detect(cv_image)
        annotated = self.detector.draw_results(results)

        # publicar imagem anotada
        if self.publish_annotated:
            try:
                out_msg = cv2_to_msg(annotated, self.bridge)
                out_msg.header = msg.header
                self.pub_image.publish(out_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'Erro ao converter imagem anotada: {e}')

    def timer_callback(self):
        color_image, depth_image = self.realsense.get_frame()
        if color_image is None:
            return

        results = self.detector.detect(color_image)
        annotated = self.detector.draw_results(results)

        if self.publish_annotated:
            try:
                out_msg = cv2_to_msg(annotated, self.bridge)
                self.pub_image.publish(out_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'Erro ao publicar imagem anotada: {e}')

    def destroy_node(self):
        # cleanup
        if hasattr(self, 'realsense'):
            try:
                self.realsense.stop()
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YOLORealsenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()