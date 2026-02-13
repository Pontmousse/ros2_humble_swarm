from setuptools import find_packages, setup

package_name = 'swarm_marvelmind'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'swarm_marvelmind.marvelmind',  # ✅ Explicitly include marvelmind.py
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
            'marvelmind_obs = swarm_marvelmind.marvelmind_obs:main',
            'hedgehog_obs= swarm_marvelmind.hedgehog_obs:main'
        ],
    },
)
