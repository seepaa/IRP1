from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'proj1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Thomas Kaseca',
    maintainer_email='112017654+ThoamsKaseca@users.noreply.github.com',
    description=('CS 4023/5023 Project 1: reactive controller and occupancy-grid '
                 'mapper for a simulated TurtleBot 4'),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'reactive_node = proj1.reactive_node:main',
            'occupancy_mapper = proj1.occupancy_mapper:main',
        ],
    },
)
