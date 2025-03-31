from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
import os



def generate_launch_description():
    map_server_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("yahboomcar_nav"),
                "launch",
                "map_cartographer_launch.py",
            )),
        )
        
    laser_to_point_node = Node(
        package='laserscan_to_point_pulisher',
        node_executable='laserscan_to_point_pulisher',
        output="screen"
    )
    return LaunchDescription(
        [
            laser_to_point_node,
            map_server_launch   
        ]
    )