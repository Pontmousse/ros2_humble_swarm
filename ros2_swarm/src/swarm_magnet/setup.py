from setuptools import find_packages, setup

package_name = 'swarm_magnet'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "gpio_control_node = swarm_magnet.gpio_control_node:main",
            "linear_consensus_node = swarm_magnet.linear_consensus_node:main"
        ],
    },
)
