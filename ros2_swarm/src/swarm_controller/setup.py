from setuptools import find_packages, setup

package_name = 'swarm_controller'

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
            #########################################################################
            'bounding_box_visualizer = swarm_controller.bounding_box_visualizer:main',

            'guidance = swarm_controller.guidance_node:main',

            #########################################################################

            'stop_all = swarm_controller.stop_all:main',
            'all_capture = swarm_controller.all_capture:main',
            'mode_supervisor = swarm_controller.mode_supervisor:main',

            'ap_auction = swarm_controller.ap_auction_node:main',
            'readiness_consensus = swarm_controller.readiness_consensus_node:main',
            'consensus_benchmark = swarm_controller.consensus_benchmark:main',

        ],
    },
)
