from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_simulation'

setup(
    name='arm_simulation',
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='gmslazzarini@gmail.com',
    description='Simulation package for a robotic arm using Webots',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 'controller = arm_simulation.controller:main',
            'hybrid_controller = arm_simulation.hybrid_controller:main',
        ],
    },
)
