from setuptools import find_packages, setup

package_name = 'keyboard_check'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'keyboard_publisher = keyboard_check.keyboard_publisher:main',
          'keyboard_listener = keyboard_check.keyboard_listener:main',
        ],
    },
)
