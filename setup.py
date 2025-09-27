from setuptools import setup

package_name = 'yolo_realsense'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['ultralytics', 'opencv-python', 'numpy', 'pyrealsense2'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    entry_points={
        'console_scripts': [
            'yolo_realsense_node = yolo_realsense.yolo_detector:main'
        ],
    },
)