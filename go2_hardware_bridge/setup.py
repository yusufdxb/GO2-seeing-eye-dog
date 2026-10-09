from setuptools import setup

package_name = "go2_hardware_bridge"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yusuf Guenena",
    maintainer_email="yusuf.a.guenena@gmail.com",
    description="Hardware adapter contract and bridge node for the GO2.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "hardware_bridge_node = go2_hardware_bridge.hardware_bridge_node:main",
            "sport_stop_watchdog_node = go2_hardware_bridge.stop_watchdog:main",
        ],
    },
)
