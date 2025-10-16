import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('yolo_realsense')
    default_model = os.path.join(pkg_share, 'models', 'yolov8n.pt')

    use_driver_node = LaunchConfiguration('use_driver_node')
    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')
    conf_threshold = LaunchConfiguration('conf_threshold')
    publish_annotated = LaunchConfiguration('publish_annotated')
    camera_topic = LaunchConfiguration('camera_topic')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_driver_node',
            default_value='True',
            description=(
                'Se True, utiliza o driver ROS2 da Intel RealSense. '
                'Se False, tenta acesso direto à RealSense ou à webcam.'
            )
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value=default_model,
            description='Caminho do modelo YOLOv8 (.pt).'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='cpu',
            description='Dispositivo para inferência (cpu ou cuda).'
        ),
        DeclareLaunchArgument(
            'conf_threshold',
            default_value='0.25',
            description='Nível mínimo de confiança para detecção YOLO.'
        ),
        DeclareLaunchArgument(
            'publish_annotated',
            default_value='True',
            description='Se True, publica a imagem anotada no tópico /yolo/annotated_image.'
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/camera/color/image_raw',
            description='Tópico da câmera (usado apenas se use_driver_node=True).'
        ),

        Node(
            package='yolo_realsense',
            executable='main',
            name='yolo_realsense_node',
            output='screen',
            parameters=[{
                'use_driver_node': use_driver_node,
                'model_path': model_path,
                'device': device,
                'conf_threshold': conf_threshold,
                'publish_annotated': publish_annotated,
                'camera_topic': camera_topic
            }]
        )
    ])
