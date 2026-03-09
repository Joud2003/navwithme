from setuptools import find_packages, setup

package_name = "nav_with_me"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="joud-ros2",
    maintainer_email="joud.salhi10@gmail.com",
    description="NavWithMe Project",
    license="Apache License 2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "move_robot = nav_with_me.robot_navigation:main",
        ],
    },
)
