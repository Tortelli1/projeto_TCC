from setuptools import setup
import os
from glob import glob

package_name = 'yolo_realsense'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=[
        'setuptools',
        'ultralytics',
        'opencv-python',
        'numpy',
        'pyrealsense2',
        'opencv-python-headless; platform_system == "Linux"'
    ],
    zip_safe=True,
    maintainer='Marco Tortelli',
    maintainer_email='marcopazinitortelli@gmail.com',
    description='YOLOv8 + Intel RealSense integration for ROS2',
    license='MIT',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    entry_points={
        'console_scripts': [
            'yolo_realsense_node = yolo_realsense.main:main',
        ],
    },
)