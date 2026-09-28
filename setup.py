from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'reactive_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'INTERFACES.md']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Thomas's world, installed from its original location (not copied
        # in the repo). The world has no external model assets.
        (os.path.join('share', package_name, 'worlds'),
            ['cust_robo_room.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kiara Nelson',
    maintainer_email='kelsoooooo03@gmail.com',
    description='Reactive TurtleBot 4 controller and occupancy-grid mapper.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'reactive_controller = reactive_robot.reactive_controller:main',
            'occupancy_mapper = reactive_robot.occupancy_mapper:main',
        ],
    },
)
