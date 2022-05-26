import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    
    config = os.path.join(
      get_package_share_directory('softhand_control_arbotix'),
      'config',
      'softhand_vision_based_grasp_param.yaml'
      )
    
    return LaunchDescription([
        
        Node(
            package='softhand_control_arbotix',
            executable='softhand_setspeed',
            name='softhand_setspeed_clientnode',
            output='screen',
            emulate_tty=True,
            parameters=[config]
        ),
        
        Node(
            package='softhand_control_arbotix',
            executable='softhand_vision_based_grasp',
            name='softhand_grasp_node',
            output='screen',
            emulate_tty=True,
            parameters=[config]
        )
    ])

