from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'toio_omni_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hijimasa',
    maintainer_email='hijimasa@gmail.com',
    description='Builds a map of the mat from the range sensor and the Position ID',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'gt_mapper = toio_omni_mapping.gt_mapper:main',
            'coverage_drive = toio_omni_mapping.coverage_drive:main',
        ],
    },
)
