import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_control',
            executable='tree_executor',
            name='uav_tree_executor',
            output='screen',
            emulate_tty=True,
            parameters=[
                # 这里可以随时把默认的 XML 替换成别的任务树！
                {'tree_xml_file': 'uav_inspection_tree.xml'}
            ]
        )
    ])