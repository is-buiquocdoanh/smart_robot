from setuptools import find_packages, setup

package_name = 'line_follow'

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
    maintainer='jetson',
    maintainer_email='doanh762003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rs485_dual_sensor_pub = line_follow.rs485_dual_sensor_pub:main',
            'line_follower_cmdvel_mag = line_follow.line_follower_cmdvel_mag:main',
        ],
    },
)
