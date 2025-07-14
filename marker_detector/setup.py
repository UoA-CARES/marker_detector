from setuptools import find_packages, setup

package_name = "marker_detector"

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
    maintainer="anyone",
    maintainer_email="henryamwilliams@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detector_node = marker_detector.detector_node:main",
            "aruco_detector_node = marker_detector.aruco_detector:main",
            "aruco_mock_publisher_node = marker_detector.aruco_mock_publisher_node:main",
        ],
    },
)
