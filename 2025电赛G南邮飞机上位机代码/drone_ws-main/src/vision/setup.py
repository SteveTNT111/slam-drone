from setuptools import find_packages, setup

package_name = 'vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='2687690230@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'color_detector_core = vision.color_detector_core:main',
        'down_camera_init = vision.down_camera_init:main',
        'down_camera = vision.down_camera:main',
        'down_camera_color = vision.down_camera_color:main',
        'kalman_filter = vision.kalman_filter:main',
        'front_pole_detection = vision.front_pole_detection:main',
        'cable_detector = vision.cable_detector:main',
        'red_pole_detection = vision.red_pole_detection:main',
        'front_camera_init_node = vision.front_camera_init_node:main',
        'detection_socket_receiver = vision.detection_socket_receiver:main',
        'vision_task_manager = vision.vision_task_manager:main',
        'plant_detection = vision.plant_detection:main',
        'code_scanner = vision.code_scanner:main',
        ],
    },
)
