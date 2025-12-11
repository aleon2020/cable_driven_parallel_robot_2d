import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


# generate_launch_description() function
# Creates and returns the ROS2 launch description with all required nodes
def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_path = os.path.join(get_package_share_directory('cdpr_2d'))
    xacro_file = os.path.join(pkg_path, 'description/robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(pkg_path, 'config', 'robot.rviz')]]
    )

    cdpr_controller_node = Node(
        package='cdpr_2d',
        executable='cdpr_controller',
        name='cdpr_controller',
        parameters=[os.path.join(pkg_path, 'config', 'params.yaml')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        robot_state_publisher_node,
        rviz2_node,
        cdpr_controller_node,
    ])
