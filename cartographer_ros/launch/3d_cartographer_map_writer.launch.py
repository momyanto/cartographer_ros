from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    ## ***** Launch arguments *****
    configuration_directory_arg = DeclareLaunchArgument('configuration_directory', default_value = FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files')
    config_file_arg = DeclareLaunchArgument('config_file', default_value = 'cart_3d_assets_writer.lua')
    urdf_filename_arg = DeclareLaunchArgument('urdf_filename', default_value = FindPackageShare("robot_description") + '/urdf/cart/cart_real.urdf')
    bag_filenames_arg = DeclareLaunchArgument('bag_filenames', default_value='')
    pose_graph_filename_arg = DeclareLaunchArgument('pose_graph_filename', default_value='')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_assets_writer',
        name='cartographer_assets_writer',
        parameters=[{'use_sim_time' : False}],
        arguments=[
            '-configuration_directory', LaunchConfiguration('configuration_directory'),
            '-configuration_basename', LaunchConfiguration('config_file'),
            '-urdf_filename', LaunchConfiguration('urdf_filename'),
            '-bag_filenames', LaunchConfiguration('bag_filenames'),
            '-pose_graph_filename', LaunchConfiguration('pose_graph_filename')],
        output='screen'
        )
    
    return LaunchDescription([
        # launch argument
        configuration_directory_arg,
        config_file_arg,
        urdf_filename_arg,
        bag_filenames_arg,
        pose_graph_filename_arg,
        # Nodes
        cartographer_node,
    ])
