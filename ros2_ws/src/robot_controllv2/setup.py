from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_controllv2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),      glob('launch/*')),
        (os.path.join('share', package_name, 'urdf'),        glob('urdf/*')),
        (os.path.join('share', package_name, 'controllers'), glob('controllers/*')),
        (os.path.join('share', package_name, 'meshes'),      glob('meshes/*')),
        (os.path.join('share', package_name, 'config'),      glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bmaric',
    maintainer_email='bmaric@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_servos_node = robot_controllv2.move_servos_node:main',
            'ik_node = robot_controllv2.ik_node:main',
        ],
    },
)
