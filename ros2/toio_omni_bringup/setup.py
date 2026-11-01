from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'toio_omni_bringup'

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
        (os.path.join('share', package_name, 'map'), glob(os.path.join('map', '*.[py][ga]*'))),
        (os.path.join('share', package_name, 'behavior_trees'),
         glob(os.path.join('behavior_trees', '*.xml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hijimasa',
    maintainer_email='hijimasa@gmail.com',
    description='Runs the toio cube driver and the omnidirectional range sensor together',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'toio_odom_bridge = toio_omni_bringup.toio_odom_bridge:main',
            'cmd_vel_deadband = toio_omni_bringup.cmd_vel_deadband:main',
        ],
    },
)
