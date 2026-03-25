from setuptools import find_packages, setup

package_name = 'go2_intent_grounding'

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
    description='Intent grounding node for GO2 seeing-eye dog.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'intent_grounding_node = go2_intent_grounding.intent_grounding_node:main',
        ],
    },
)
