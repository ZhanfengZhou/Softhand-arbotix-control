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
            'softhand_grasp_rotate = softhand_control_arbotix.softhand_grasp_rotate:main',
            'softhand_grasp_lateral_rotate = softhand_control_arbotix.softhand_grasp_lateral_rotate:main',
            'softhand_setspeed = softhand_control_arbotix.softhand_setspeed_servicecall:main',
            'softhand_grasp_multiobjects = softhand_control_arbotix.softhand_control_grasp_multiobjects:main',
            'softhand_control_grasp_small_objects = softhand_control_arbotix.softhand_control_grasp_small_objects:main',
            'softhand_poker_manipulate = softhand_control_arbotix.softhand_poker_manipulate:main',
            'softhand_manipulate_benchmark = softhand_control_arbotix.softhand_manipulate_benchmark:main',
            'softhand_grasp_lateral_spraying = softhand_control_arbotix.softhand_grasp_lateral_spraying:main',
            'softhand_vision_based_grasp = softhand_control_arbotix.softhand_vision_based_grasp:main',
        ],
    },
)
