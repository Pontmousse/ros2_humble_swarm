from setuptools import find_packages, setup

package_name = 'swarm_comms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elghali',
    maintainer_email='elghali@todo.todo',
    description='Periodic UDP state broadcast and neighbour table for the swarm.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_broadcaster = swarm_comms.state_broadcaster:main',
            'sniffer = swarm_comms.sniffer:main',
        ],
    },
)
