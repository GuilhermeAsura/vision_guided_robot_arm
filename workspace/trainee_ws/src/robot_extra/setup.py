from setuptools import setup
import os
from glob import glob 

package_name = 'robot_extra'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giovanna',
    maintainer_email='example@email.com',
    description='Extra: Depth estimator + IK',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_estimator_node = robot_extra.depth_estimator_node:main',
            'robot_controller_node = robot_extra.robot_controller_node:main',
            'ik_controller_node = robot_extra.ik_controller_node:main',
        ],
    },
)