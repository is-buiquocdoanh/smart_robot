from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'auto_docking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='doanh762003@gmail.com',
    description='Auto docking (navigate to charging pose + ultrasonic reverse docking)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'docking_manager = auto_docking.docking_manager:main',
        ],
    },
)
