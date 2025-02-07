from setuptools import find_packages
from setuptools import setup

setup(
    name='wadar_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('wadar_interfaces', 'wadar_interfaces.*')),
)
