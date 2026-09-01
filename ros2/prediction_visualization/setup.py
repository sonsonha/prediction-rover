from setuptools import find_packages, setup

package_name = "prediction_visualization"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/prediction_visualization.yaml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/prediction_visualization.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Landfill Rover Team",
    maintainer_email="dev@example.com",
    description="RViz visualization for Prediction canonical topics (viz only)",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "prediction_visualization_node = prediction_visualization.node:main",
        ],
    },
)
