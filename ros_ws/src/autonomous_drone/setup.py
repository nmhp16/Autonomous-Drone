from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'autonomous_drone'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('autonomous_drone/perception/*.npz')),
        (os.path.join('share', package_name, 'config'), glob('autonomous_drone/bridges/*.npz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Nguyen Pham, Aaron Mundanilkunathil',
    author_email='nmhieu.pham@gmail.com, aaron.mundanilkunathil@sjsu.edu',
    description='Autonomous drone with object detection, avoidance, and following',
    license='GPL-3.0-or-later',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_interface = autonomous_drone.node_interface:main',
            'object_detector = autonomous_drone.perception.object_detector:main',
            'object_avoidance = autonomous_drone.perception.object_avoidance:main',
            'object_following = autonomous_drone.perception.object_following:main',
            'vslam_node = autonomous_drone.perception.vslam_node:main',
            'sim_bridge = autonomous_drone.bridges.sim_bridge:main',
            'udp_custom_receiver = autonomous_drone.bridges.udp_custom_receiver:main',
        ],
    },
)
