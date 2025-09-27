# launch/yolo_realsense_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Se você já tiver realsense2_camera instalado, descomente / ative este Node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        output='screen',
        # você pode passar parâmetros aqui se necessário
    )

    yolo_node = Node(
        package='yolo_realsense',
        executable='yolo_realsense_node',
        name='yolo_realsense_node',
        output='screen',
        parameters=[
            {'model_path': 'models/yolov8n.pt'},   # ajuste se for best.pt
            {'device': 'cpu'},
            {'target_classes': []},                # [] = todas as classes
            {'conf_threshold': 0.25}
        ]
    )

    # Retire realsense_node se preferir rodar driver separadamente
    ld.add_action(realsense_node)
    ld.add_action(yolo_node)

    return ld