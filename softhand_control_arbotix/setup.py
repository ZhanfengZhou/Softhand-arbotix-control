from setuptools import setup
import os
from glob import glob

package_name = 'softhand_control_arbotix'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhanfeng',
    maintainer_email='zhanfeng.zhou@mail.utoronto.ca',
    description='Publisher sending messages to arbotix topic in ROS1 using the ROS1_bridge',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'softhand_grasp = softhand_control_arbotix.softhand_control_grasp:main',
            'softhand_fingers_move2points = softhand_control_arbotix.softhand_fingers_move2points:main',
            'softhand_setspeed = softhand_control_arbotix.softhand_setspeed_servicecall:main',
        ],
    },
)
