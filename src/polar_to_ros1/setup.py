from setuptools import find_packages, setup

package_name = 'polar_to_ros1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seamate-docker',
    maintainer_email='pk421933605@gmail.com',
    description='ROS2 node to send /Navtech/Polar messages to ROS1',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'polar_to_ros1_node = polar_to_ros1.polar_to_ros1_node:main',
        ],
    },
)
