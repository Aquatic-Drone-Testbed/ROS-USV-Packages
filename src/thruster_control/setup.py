from setuptools import find_packages, setup

package_name = 'thruster_control'

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
    maintainer='seamate',
    maintainer_email='dna@ucsb.edu',
    description='Control Thruster Operation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'thruster_control = thruster_control.thruster:main',
        'joystick_control = thruster_control.joystick:main'
        ],
},
)
