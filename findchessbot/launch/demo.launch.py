#!/usr/bin/env python3
import os
from time import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



# def generate_launch_description():

#     use_sim_time = LaunchConfiguration('use_sim_time', default='false')

#     urdf_file_name = 'urdf/findchessbot.urdf'
#     urdf = os.path.join(
#         get_package_share_directory('findchessbot'),
#         urdf_file_name)
#     with open(urdf, 'r') as infp:
#         robot_desc = infp.read()

#     rviz_file_name = 'view.rviz'
#     rviz_config = os.path.join(
#         get_package_share_directory('findchessbot'),
#         rviz_file_name)
    
#         # 
#     return LaunchDescription([
#         # DeclareLaunchArgument(
#         #     'use_sim_time',
#         #     default_value='false',
#         #     description='Use simulation (Gazebo) clock if true'),
        
#         # Node(
#         #     package='rviz2',
#         #     executable='rviz2',
#         #     name='rviz2',
#         #     arguments=['-d', rviz_config],
#         #     output='screen'),

#         # Node(
#         #     package='robot_state_publisher',
#         #     executable='robot_state_publisher',
#         #     name='robot_state_publisher',
#         #     output='screen',
#         #     parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
#         #     arguments=[urdf]),

#         # Node(
#         #     package='findchessbot',
#         #     executable='python_node.py',
#         #     name='python_node',
#         #     output='screen'),

#         # Node(
#         #     package='findchessbot',
#         #     executable='state_publisher.py',
#         #     name='state_publisher',
#         #     output='screen'),

#         Node(
#             package='findchessbot',
#             executable='input_chessboard_rpm.py',
#             name='input_chessboard_rpm',
#             output='screen'),
        
#     ])

def generate_launch_description():
    # Get the launch directory
    findchessbot_dir = get_package_share_directory('findchessbot')
    launch_dir = os.path.join(findchessbot_dir, 'launch')
    rviz_file_name = 'view.rviz'
    rviz_file_path = os.path.join(get_package_share_directory('findchessbot'), 'rviz', rviz_file_name)
    print(rviz_file_path)
    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # urdf_file_name = 'urdf/findchessbot.urdf'
    # urdf = os.path.join(
    #     get_package_share_directory('findchessbot'),
    #     urdf_file_name)
    # with open(urdf, 'r') as infp:
    #     robot_desc = infp.read()

    pkg_share = get_package_share_directory('findchessbot')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'findchessbot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    params = {'robot_description': robot_desc}

    rviz_Node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_ros2',
        arguments=['-d', rviz_file_path],
        output='screen')



    findchessbot_input_chessboard_rpm_Node = Node(
        package='findchessbot',
        executable='input_chessboard_rpm.py',
        name='input_chessboard_rpm',
        output='screen')

    findchessbot_state_pubilsher_Node = Node(
        package='findchessbot',
        executable='findchessbot_state_publisher.py',
        name='findchessbot_state_publisher',
        output='screen')

    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=[params])

    print(robot_state_publisher)
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(robot_state_publisher)
    ld.add_action(findchessbot_input_chessboard_rpm_Node)
    ld.add_action(findchessbot_state_pubilsher_Node)
    # ld.add_action(rviz_Node)

    return ld