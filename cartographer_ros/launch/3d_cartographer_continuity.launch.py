"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

# 地図有り、自己位置3dcartograher 経路計画nav2のためのlaunch
# navigation2_cartographer.launch.py->bringup_launch.py ->
# -> cartographer_ros/cartographer_node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.actions import Shutdown
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False')
    load_state_filename_arg = DeclareLaunchArgument('load_state_filename')

    # ***** File paths ******
    # urdf_path = get_package_share_directory('robot_description') + '/urdf/cart/cart.urdf'
    # with open(urdf_path, 'r') as infp:
    #     robot_desc = infp.read()

    # ***** Nodes *****
    # robot_state_publisher_node = Node(
    #     package = 'robot_state_publisher',
    #     executable = 'robot_state_publisher',
    #     parameters=[
    #         {'robot_description': robot_desc},
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    #     output = 'screen'
    #     )

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments = [
            '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
            '-configuration_basename', 'cart_robot.lua',
            '-load_state_filename', LaunchConfiguration('load_state_filename'),
            ],
        remappings = [
            # ('points2', 'livox/lidar'),
            ('points2_1', 'livox/lidar_right'),
            ('points2_2', 'livox/lidar_left'),
            # ('points2', 'unilidar/cloud'),
            # ('points2_1', 'livox/lidar'),
            # ('points2_2', 'unilidar/cloud'),
 
            ('imu', 'imu/data_calib'),
            # ('odom', 'adjusted_odom')
            ],
        output = 'screen'
        )

    # nav2に直接mapを入力する場合、cartographerはmapをpubしなくて良い(pure localiczation)
    # cartographer_occupancy_grid_node = Node(
    #     package = 'cartographer_ros',
    #     executable = 'cartographer_occupancy_grid_node',
    #     parameters = [
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')},
    #         {'resolution': 0.05}],
    #     )
    
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        on_exit = Shutdown(),
        arguments = ['-d', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/cart_3d.rviz'],
        parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        load_state_filename_arg,
        # Nodes
        # robot_state_publisher_node,
        cartographer_node,
        # cartographer_occupancy_grid_node,
        rviz_node,
    ])
