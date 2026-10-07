import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'social_nav_safety'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hung',
    maintainer_email='bhung1175@gmail.com',
    description='Lidar safety layer: last guard between velocity_smoother and the motor driver',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_safety_node1 = social_nav_safety.lidar_safety_node1:main',
        ],
    },
)
