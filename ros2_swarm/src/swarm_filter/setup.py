from setuptools import find_packages, setup

package_name = 'swarm_filter'

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
        	"kalman_filter = swarm_filter.kalman_filter:main",
            "pose_publisher = swarm_filter.pose_publisher:main",
            "mm_filtered = swarm_filter.mm_filtered:main",
            "aruco_filter = swarm_filter.aruco_filter:main",
        ],
    },
)
