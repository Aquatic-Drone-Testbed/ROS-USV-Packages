from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slam_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name), ['slam_toolbox_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seamate-docker',
    maintainer_email='baronyoung2001@gmail.com',
    description='Package for SLAM Toolbox configuration',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
