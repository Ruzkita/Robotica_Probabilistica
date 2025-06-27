from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ros2_gazeboworld'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('lib/' + package_name, glob('scripts/*.py')),  
        ('share/' + package_name + '/world', glob('world/*.sdf')),
        ('share/' + package_name + '/models/meu_robo', glob('models/meu_robo/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seu Nome',
    maintainer_email='seuemail@example.com',
    description='Descrição do pacote',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_bridge=ros2_gazeboworld.ros2_bridge:main',
        ],
    },
)
