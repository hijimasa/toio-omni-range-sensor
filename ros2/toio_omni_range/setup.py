from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'toio_omni_range'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hijimasa',
    maintainer_email='hijimasa@gmail.com',
    description='ROS 2 driver for the toio omnidirectional range sensor',
    license='Apache-2.0',
    # colcon needs a declared pytest dependency to run tests with pytest
    # instead of unittest (which collects nothing here)
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'omni_range_node = toio_omni_range.omni_range_node:main',
        ],
    },
)
