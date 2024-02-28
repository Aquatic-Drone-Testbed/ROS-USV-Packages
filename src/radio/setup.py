from setuptools import find_packages, setup

package_name = 'radio'

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
    maintainer_email='jungmaxwell@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'receiver_udp = radio.receiver_udp:main',
            'sender_udp = radio.sender_udp:main',
        ],
    },
)
