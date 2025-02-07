from setuptools import find_packages
from setuptools import setup

setup(
    name='inertial_sense_ros2',
    version='1.1.1',
    packages=find_packages(
        include=('inertial_sense_ros2', 'inertial_sense_ros2.*')),
)
