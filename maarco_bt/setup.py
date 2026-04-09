from setuptools import find_packages, setup
import hashlib

package_name = 'maarco_bt'



setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='patchy',
    maintainer_email='ptedstoryv2@gmail.com',
    description='ROS2 Control Stack for MAARCO',
    license="aa185e3664228265cdcf30da417f8aa48b5a86d3dbd4d8088ab6a1cc4b77b3f3",
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        "console_scripts": [
            "heading_bt_node = maarco_bt.heading_bt_node:main",
        ],
    },
)
