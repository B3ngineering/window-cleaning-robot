from setuptools import setup
import os
from glob import glob

package_name = 'pulley_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), 
            glob('urdf/*.urdf') + glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), 
            glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Pulley system with DC motor for Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_motor = pulley_package.control_motor:main',
            'cable_physics = pulley_package.cable_physics:main',
            'motor_control_gui = pulley_package.motor_control_gui:main',
        ],
    },
)