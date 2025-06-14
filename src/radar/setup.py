from setuptools import find_packages, setup

package_name = 'radar'

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
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quantum = radar.quantum:main',
            'slam = radar.slam:main',
            'test = radar.test:main',
            'quantum_chain = radar.quantum_chain:main',
            'quantum_keepalive = radar.quantum_keepalive:main',
            'quantum_keepalive_chain = radar.quantum_keepalive_chain:main',
        ],
    },
)
