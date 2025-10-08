from setuptools import find_packages
from setuptools import setup

setup(
    name='tfg_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('tfg_interfaces', 'tfg_interfaces.*')),
)
