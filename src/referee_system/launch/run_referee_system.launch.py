from launch import LaunchDescription ,actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess,RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    referee_system_dir = get_package_share_directory('referee_system')
    referee_system_yaml_path = os.path.join(referee_system_dir,'launch','param.yaml')
    config_path = LaunchConfiguration('config_path')
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=referee_system_yaml_path,
        description='Yaml config file path'
    )
    t1 =Node(
        package="referee_system",
        executable="run_referee_system",
        parameters=[config_path],
        )
    aerial_map_frame_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    output="log" ,
    arguments=["-5.91765", "-6.88762", "0", "0", "0", "0", "map", "aerial_map"]
    ) 
    return LaunchDescription([
        declare_config_path_cmd,
        t1,
        aerial_map_frame_tf,
    ])