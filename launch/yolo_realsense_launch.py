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
    )

    yolo_node = Node(
        package='yolo_realsense',
        executable='yolo_realsense_node',
        name='yolo_realsense_node',
        output='screen',
        parameters=[
            {'use_driver_node': True},
            {'model_path': default_model},
            {'device': 'cpu'},
            {'conf_threshold': 0.25},
            {'publish_annotated': True},
            {'camera_topic': '/camera/color/image_raw'},
        ]
    )

    ld = LaunchDescription()
    ld.add_action(realsense_node)
    ld.add_action(yolo_node)

    return ld