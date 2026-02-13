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
            'pointing_guidance1 = swarm_controller.pointing_guidance1:main',
            'pointing_guidance2 = swarm_controller.pointing_guidance2:main',
            'pointing_guidance3 = swarm_controller.pointing_guidance3:main',

            'position_guidance1 = swarm_controller.position_guidance1:main',
            'position_guidance2 = swarm_controller.position_guidance2:main',
            'position_guidance3 = swarm_controller.position_guidance3:main',

            #########################################################################

            'pointing_controller = swarm_controller.pointing_controller:main',
            'ep_proportional_controller = swarm_controller.ep_proportional_controller:main',
            'cp_proportional_controller = swarm_controller.cp_proportional_controller:main',

            #########################################################################

            'stop_all = swarm_controller.stop_all:main',
            'all_capture = swarm_controller.all_capture:main',
            
        ],
    },
)