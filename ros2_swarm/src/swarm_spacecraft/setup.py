from setuptools import find_packages, setup


package_name = "swarm_spacecraft"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="elghali",
    maintainer_email="elghali@todo.todo",
    description="Virtual planar spacecraft dynamics rendered by a mobile robot",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "virtual_spacecraft = swarm_spacecraft.virtual_spacecraft_node:main",
        ],
    },
)
