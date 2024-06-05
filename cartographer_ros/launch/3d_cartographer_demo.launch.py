
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown

def generate_launch_description():
    ## ***** Launch arguments *****
    bag_filename_arg = DeclareLaunchArgument('bag_filename')

    ## ***** Nodes *****
    cartographer_3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(FindPackageShare('cartographer_ros').find('cartographer_ros') + '/launch/3d_cartographer.launch.py'),
        launch_arguments = {'use_sim_time': 'True'}.items()
        )

    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        on_exit = Shutdown(),
        arguments = ['-d', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/cart_3d.rviz'],
        parameters = [{'use_sim_time': True}],
    )

    ros2_b/

    return LaunchDescription([
        # Launch arguments
        bag_filename_arg,
        # Nodes
        cartographer_3d_launch,
        rviz_node,
        ros2_bag_play_cmd
    ])
