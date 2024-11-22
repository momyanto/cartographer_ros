
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ## ***** Launch arguments *****
    bag_filename_arg = DeclareLaunchArgument('bag_filename')

    ## ***** Nodes *****
    cartographer_3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(FindPackageShare('cartographer_ros').find('cartographer_ros') + '/launch/3d_cartographer.launch.py'),
        launch_arguments = {'use_sim_time': 'True'}.items()
        )

    qos_override_json = DeclareLaunchArgument(
        'qos_override_file',
        default_value=os.path.join(
            get_package_share_directory('cartographer_ros'),
            'configuration_files', 
            'qos_overrides.json'
        ),
        description='Path to the QoS override file'
    )

    # rviz_node = Node(
    #     package = 'rviz2',
    #     executable = 'rviz2',
    #     on_exit = Shutdown(),
    #     arguments = ['-d', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/cart_3d.rviz'],
    #     parameters = [{'use_sim_time': True}],
    # )

    ros2_bag_play_cmd = ExecuteProcess(
        cmd = ['ros2', 'bag', 'play', LaunchConfiguration('bag_filename'), '--clock',
                '--rate', '3.0',
                '--start-offset', '60',
                '--topic',
                # '/livox/lidar', 
                # '/livox/lidar_left', 
                # '/livox/lidar_right', 
                '/livox/lidar_192_168_1_163',
                # '/livox/lidar_192_168_171_126',
                # '/livox/lidar_192_168_171_127',
                # '/imu/data_raw', 
                # '/imu/data_calib',
                '/imu_ms2',
                '/tf_static', 
                '/odom',
                '/tf', 
                # '/rosout',
                # '--qos-profile-overrides-path', LaunchConfiguration('qos_override_file')
                ],
        name = 'rosbag_play',
    )
    
    return LaunchDescription([
        # Launch arguments
        bag_filename_arg,
        qos_override_json,
        # Nodes
        cartographer_3d_launch,
        # rviz_node,
        ros2_bag_play_cmd,
    ])

