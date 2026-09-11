from setuptools import setup

# Directory name is `features/` (the BDD suite is transversal - its scenarios
# exercise central_hub, sensor, patient and system_monitor together, so it
# belongs beside them, not inside any one of them). The ROS package name is
# `bsn_bdd`: package names are global to the ROS ecosystem, and `features`
# would be far too generic in `ros2 pkg list`. colcon takes the name from
# package.xml, not from the directory, so the two are free to differ.
package_name = "bsn_bdd"

setup(
    name=package_name,
    version="0.0.0",
    # No Python packages: harness.py lives in test/ and is imported directly by
    # the harnesses next to it, and steps/ is test input, not runtime code.
    # Nothing here is importable from outside, so nothing is installed except
    # the ament marker below.
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DockerUser",
    maintainer_email="gabrielbfranca@gmail.com",
    description=(
        "BDD suite for the BSN: .feature specs at the root, their pytest-bdd "
        "step definitions in steps/, and launch_testing harnesses in test/ that "
        "bring up a per-feature ROS2 stack and run the steps against it."
    ),
    license="TODO: License declaration",
    tests_require=["pytest", "launch_testing"],
)
