from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
import os



def generate_launch_description():

    map_yaml_file = LaunchConfiguration('map')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join('/root/data/', 'maps', 'map.yaml'),
        description='Full path to map file to load')
    
    navigation_teb_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("yahboomcar_nav"),
                "launch",
                "navigation_dwa_launch.py",
            )),
            launch_arguments={'map': map_yaml_file}.items()
        )
        
    laser_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("yahboomcar_nav"),
                "launch",
                "laser_bringup_launch.py",
            )),
        )

    laser_to_point_node = Node(
        package='laserscan_to_point_pulisher',
        node_executable='laserscan_to_point_pulisher',
        output="screen"
    )

    robot_pose_node = Node(
        package='robot_pose_publisher',
        node_executable='robot_pose_publisher',
        output="screen"
    )
    
    return LaunchDescription(
        [
            declare_map_yaml_cmd,
            laser_to_point_node,
            laser_bringup,
            navigation_teb_server,
            robot_pose_node   
        ]
    )