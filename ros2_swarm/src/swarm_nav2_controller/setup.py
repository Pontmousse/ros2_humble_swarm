from setuptools import find_packages, setup

package_name = 'swarm_nav2_controller'

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
    description='Tracking controllers for the RoboMaster swarm',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #########################################################################
            'nav2_pff = swarm_nav2_controller.nav2_pff:main', # P-error position + feedforward velocity
            'nav2_pidff = swarm_nav2_controller.nav2_pidff:main', # PID position + feedforward velocity
            'nav2_dwb_path = swarm_nav2_controller.nav2_dwb_path:main', # One-pose moving goal for DWB
            #########################################################################
        ],
    },
)
