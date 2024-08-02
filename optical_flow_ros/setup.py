from setuptools import find_packages, setup

package_name = "optical_flow_ros"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "lib/" + package_name,
            [package_name + "/flow_opencv.py", package_name + "/optical_flow.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="smruti",
    maintainer_email="suresh.sm@northeastern.edu",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "flow_publisher = scripts.flow_publisher:main",
        ],
    },
)
