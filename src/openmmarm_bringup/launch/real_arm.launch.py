#!/usr/bin/env python3
"""
OpenMMARM 实机模式启动文件

启动完整的实机控制环境：
  1. openmmarm_hw（ros2_control 硬件接口，通过 IOUDP 连接真实机械臂）
  2. move_group（MoveIt 运动规划）
  3. robot_state_publisher（TF 发布）
  4. rviz2（可选）

注意：真机模式需要先将 control_config.yaml 中 communication 改为 "UDP" 并配置 MCU 地址。
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, file_path):
    """从包中加载 YAML 文件"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception:
        return None


def generate_launch_description():
    # 包路径
    description_pkg = get_package_share_directory('openmmarm_description')
    moveit_config_pkg = get_package_share_directory('openmmarm_moveit_config')

    # Launch 参数
    use_rviz = LaunchConfiguration('use_rviz')
    use_moveit = LaunchConfiguration('use_moveit')

    # 使用 xacro 处理 robot.xacro
    xacro_file = os.path.join(description_pkg, 'urdf', 'robot.xacro')
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # robot_state_publisher 节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # openmmarm_hw 启动（ros2_control 硬件接口）
    hw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('openmmarm_hw'),
                'launch',
                'openmmarm_hw.launch.py',
            ])
        ]),
    )

    # MoveIt move_group 启动
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('openmmarm_moveit_config'),
                'launch',
                'move_group.launch.py',
            ])
        ]),
        condition=IfCondition(use_moveit),
    )

    # 加载 SRDF 和 kinematics 配置（RViz MotionPlanning 插件需要）
    srdf_file = os.path.join(moveit_config_pkg, 'config', 'openmmarm.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics_yaml = load_yaml('openmmarm_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # RViz2 节点
    rviz_config_file = os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='启动 RViz2',
        ),
        DeclareLaunchArgument(
            'use_moveit',
            default_value='true',
            description='启动 MoveIt move_group',
        ),
        robot_state_publisher_node,
        hw_launch,
        moveit_launch,
        rviz_node,
    ])
