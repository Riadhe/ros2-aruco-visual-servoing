import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'erl1_treasure_hunt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ba7rouch',
    maintainer_email='bahririadhe13@gmail.com',
    description='ArUco treasure hunt: visual servoing, rotation and state machine nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_servoing = erl1_treasure_hunt.visual_servoing:main',
            'rotation = erl1_treasure_hunt.rotation:main',
            'state_machine = erl1_treasure_hunt.state_machine:main',
        ],
    },
)