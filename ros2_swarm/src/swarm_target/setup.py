import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'swarm_target'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elghali',
    maintainer_email='elghali@todo.todo',
    description='Target/landmark providers: virtual scene simulator and the '
               'ArUco-fused landmark/target memory estimator.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'virtual_scene = swarm_target.virtual_scene:main',
            'landmark_estimator = swarm_target.landmark_estimator:main',
        ],
    },
)
