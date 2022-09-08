import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    
    config = os.path.join(
      get_package_share_directory('softhand_control_arbotix'),
      'config',
      'softhand_vision_based_manipulation_PID_step23_param.yaml'
      )
    
    return LaunchDescription([
        
        Node(
            package='softhand_control_arbotix',
            executable='softhand_vision_based_manipulation_pid_step2',
            name='softhand_manipulation_node',
            output='screen',
            emulate_tty=True,
            parameters=[config]
        )
    ])

