# launch/yolo_realsense_launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('yolo_realsense')
    default_model = os.path.join(pkg_share, 'models', 'yolov8n.pt')

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        output='screen',
        # se quiser, adicione parâmetros aqui
    )

    yolo_node = Node(
        package='yolo_realsense',
        executable='yolo_realsense_node',
        name='yolo_realsense_node',
        output='screen',
        parameters=[
            {'use_driver_node': True},                 # True = usa driver realsense2_camera (subscribe)
            {'model_path': default_model},
            {'device': 'cpu'},
            {'conf_threshold': 0.25},
            {'publish_annotated': True},               # publica imagens anotadas em /yolo/annotated_image
            {'camera_topic': '/camera/color/image_raw'},# topic a escutar quando use_driver_node=True
        ]
    )

    ld = LaunchDescription()
    # adicione realsense_node se desejar que o launch também inicie o driver
    ld.add_action(realsense_node)
    ld.add_action(yolo_node)

    return ld