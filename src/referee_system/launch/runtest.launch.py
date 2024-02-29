from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess,RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # 1.创建两个 turtlesim_node 节点
    t1 = Node(package="referee_system",executable="mytest")

    return LaunchDescription([t1])