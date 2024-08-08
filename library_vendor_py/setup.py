from setuptools import setup
from setuptools.command.build import build
import logging


class BuildMessage(build):
    def run(self):
        logging.log(logging.WARN, "\nThis package currently does nothing, please install all dependencies manually using 'pip' or use the 'setup.sh' script.\n")
        build.run(self)


package_name = 'library_vendor_py'

setup(
    name=package_name,
    version='0.0.1',
    description='Python dependency library vendor for robot-commander.',
    author='Tomas Horelican',
    maintainer='Tomas Horelican',
    maintainer_email='tomas.horelican@vut.cz',
    license='MIT',
    packages=[package_name],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: MIT License"
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    cmdclass={'build': BuildMessage},
)
