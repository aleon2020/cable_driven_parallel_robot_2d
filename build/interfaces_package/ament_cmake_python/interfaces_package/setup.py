from setuptools import find_packages
from setuptools import setup

setup(
    name='interfaces_package',
    version='0.0.1',
    packages=find_packages(
        include=('interfaces_package', 'interfaces_package.*')),
)
