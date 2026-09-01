from glob import glob

from setuptools import find_packages, setup

package_name = "decision_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Landfill Rover Team",
    maintainer_email="dev@example.com",
    description="Policy-neutral Decision evidence aggregation (V0)",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "decision_evidence_node = decision_ros.node:main",
        ],
    },
)
