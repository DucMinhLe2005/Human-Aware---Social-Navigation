from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'social_nav_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install the model weights (.pt and .onnx) so the launch file finds them in share/.
        (os.path.join('share', package_name, 'weights'),
            glob('weights/*.pt') + glob('weights/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hung',
    maintainer_email='bhung1175@gmail.com',
    description='YOLO pose + depth 3D human detection, optimised for CPU-only computers',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_pose_node1 = social_nav_perception.yolo_pose_node1:main',
        ],
    },
)
