import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 行为树大脑
        Node(
            package='uav_control', executable='tree_executor',
            name='uav_tree_executor', output='screen',
            parameters=[{'tree_xml_file': 'uav_inspection_tree.xml'}]
        ),
        # 🌟 TF 空间树广播中心 (带参数注入)
        Node(
            package='uav_control', executable='uav_tf_broadcaster',
            name='uav_tf_broadcaster', output='screen',
            parameters=[{
                'camera_offset_x': 0.15,  # 比如换了机架，相机前突0.15米
                'camera_offset_y': 0.0,
                'camera_offset_z': -0.05  # 相机下沉5厘米
            }]
        )
    ])