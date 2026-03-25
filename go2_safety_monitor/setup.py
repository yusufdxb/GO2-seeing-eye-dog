from setuptools import find_packages, setup

package_name = 'go2_safety_monitor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yusuf Guenena',
    maintainer_email='yusuf.a.guenena@gmail.com',
    description='Safety monitor node for GO2 seeing-eye dog.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'safety_monitor_node = go2_safety_monitor.safety_monitor_node:main',
        ],
    },
)
