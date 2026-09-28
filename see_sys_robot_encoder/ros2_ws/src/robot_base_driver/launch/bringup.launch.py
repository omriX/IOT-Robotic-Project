# bringup.launch.py
#
# robot_state_publisher + driver_node + optional rviz2. The micro-ROS agent
# itself stays a separate docker-compose service (agent-udp/agent-serial),
# not part of this launch file -- it needs to keep running independently of
# whatever's being restarted on the driver/rviz side.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration('rviz')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('robot_base_driver'), 'urdf', 'robot_base.urdf.xacro'])
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('robot_base_driver'), 'config', 'robot_base.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch rviz2 alongside the driver.'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': ParameterValue(
                Command(['xacro ', xacro_file]), value_type=str)}],
        ),
        Node(
            package='robot_base_driver',
            executable='driver_node',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
