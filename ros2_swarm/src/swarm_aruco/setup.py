from setuptools import find_packages, setup

package_name = 'swarm_aruco'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'swarm_aruco.aruco',  # ✅ Explicitly include aruco.py
    ],
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
            'aruco_detector = swarm_aruco.aruco_detector:main',
            'video_processor_h264 = swarm_aruco.video_processor_h264:main',
            'video_processor_color = swarm_aruco.video_processor_color:main'
        ],
    },
)
