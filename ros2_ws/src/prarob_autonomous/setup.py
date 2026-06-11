import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'prarob_autonomous'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True,
    maintainer='sem16',
    maintainer_email='nikola.digiusto@gmail.com',
    description='Autonomous drawing orchestrator and board calibration for the '
                'PRAROB seminar robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_draw_node = prarob_autonomous.autonomous_draw_node:main',
            'board_calibration_node = prarob_autonomous.board_calibration_node:main',
        ],
    },
)
