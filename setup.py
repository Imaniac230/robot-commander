from setuptools import setup
from glob import glob
from pathlib import Path
import os

package_name = 'robot_commander'
with open(Path(__file__).resolve().parent / 'README.md', encoding='utf-8') as f: long_description = f.read()

setup(
    name=package_name,
    version='0.0.1',
    description='Bridging generative AI with ROS2 for smart robot agents.',
    author='Tomas Horelican',
    maintainer='Tomas Horelican',
    maintainer_email='tomas.horelican@vut.cz',
    license='MIT',
    long_description=long_description,
    long_description_content_type='text/markdown',
    packages=[package_name],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: MIT License"
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = robot_commander.test_node:main',
            'agent_node = robot_commander.agent_node:main',
        ],
    },
)
