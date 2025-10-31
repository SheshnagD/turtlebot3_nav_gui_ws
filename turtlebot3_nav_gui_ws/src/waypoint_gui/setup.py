from setuptools import setup
import os
from glob import glob

package_name = 'waypoint_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ats',
    maintainer_email='you@example.com',
    description='Waypoint GUI for Nav2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gui_node = waypoint_gui.gui_node:main',
        ],
    },
)

