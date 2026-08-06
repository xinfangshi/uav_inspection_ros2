import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 🧠 行为树大脑
        Node(
            package='uav_control', 
            executable='tree_executor',
            name='uav_tree_executor', 
            output='screen',
            parameters=[{
                'tree_xml_file': 'uav_inspection_tree.xml',
                'use_sim_time': True  # 🔥 强制对齐 Gazebo 仿真时间！
            }]
        ),
        # 🌟 TF 空间树广播中心 (带参数注入)
        Node(
            package='uav_control', 
            executable='uav_tf_broadcaster',
            name='uav_tf_broadcaster', 
            output='screen',
            parameters=[{
                'camera_offset_x': 0.15,
                'camera_offset_y': 0.0,
                'camera_offset_z': -0.05,
                'use_sim_time': True  # 🔥 强制对齐 Gazebo 仿真时间！
            }]
        )
        
        # ⚠️ 极其重要提示：
        # 如果你的视觉节点 (camera_subscriber) 也是写在某个 launch 文件里的，
        # 请务必也在它的 parameters 里加上 {'use_sim_time': True} ！
        # 如果你是在终端里手动 run 的，记得加上: --ros-args -p use_sim_time:=true
    ])