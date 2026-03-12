from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'semantic_sensor'

setup(
    name=package_name,
    version='2.1.0',
    packages=find_packages(include=[package_name, package_name + ".*"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lorenzo Terenzi',
    maintainer_email='lorenzoterenzi96@gmail.com',
    description='Semantic image and semantic pointcloud publishers for elevation_mapping_cupy',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pointcloud_node = semantic_sensor.pointcloud_node:main',
            'image_node = semantic_sensor.image_node:main',
        ],
    },
)
