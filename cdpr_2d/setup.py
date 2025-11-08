from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'cdpr_2d'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join(
            'share/' + package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py'))),
        (os.path.join(
            'share/' + package_name, 'description'),
            glob(os.path.join('description', '*.*'))),
        (os.path.join(
            'share/' + package_name, 'description/urdf'),
            glob(os.path.join('description/urdf', '*.*'))),
        (os.path.join(
            'share/' + package_name, 'config'),
            glob(os.path.join('config', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aalbeerto-02',
    maintainer_email='a.leon.2020@alumnos.urjc.es',
    description='Cable Driven Parallel Robot in 2 Dimensions (cdpr_2d)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cdpr_controller = cdpr_2d.cdpr_controller:main',
            'robot_controller = cdpr_2d.robot_controller:main',
            'version1_controller = cdpr_2d.version1_controller:main',
            'version2_controller = cdpr_2d.version2_controller:main',
            'version3_controller = cdpr_2d.version3_controller:main'
        ],
    },
)
