"""启动海康驱动，可选打开 RViz；相机参数统一从 YAML 读取。

ros2 launch hk_camera_driver camera.launch.py rviz:=true
ros2 launch hk_camera_driver camera.launch.py params_file:=/absolute/path/camera.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 安装后的 share 目录，不依赖当前工作目录。
    share = get_package_share_directory('hk_camera_driver')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(share, 'config', 'camera.yaml'),
            description='Absolute path to the camera ROS parameter YAML file'),
        DeclareLaunchArgument(
            'rviz', default_value='false', choices=['true', 'false'],
            description='Start RViz image preview; false for baseline rate measurements'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(share, 'rviz', 'camera.rviz'),
            description='Absolute path to an RViz configuration file'),
        # 节点名匹配 YAML 顶层；不重复设置相机参数，以免覆盖 YAML。
        Node(package='hk_camera_driver', executable='hk_camera_node', name='hk_camera_node',
             output='screen', parameters=[LaunchConfiguration('params_file')]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
