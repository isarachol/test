import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node

import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Process the URDF file from XACRO
    pkg_path = os.path.join(get_package_share_directory('bearing_formation_control'))
    xacro_file = os.path.join(pkg_path,'description','robot_car.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file,
                                        ' use_ros2_control:=', use_ros2_control])

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    rviz_config = os.path.join(
        get_package_share_directory('bearing_formation_control'), 'config',
        'xmobile_agent.rviz'
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true',),
        node_robot_state_publisher,
        Node(
            package='bearing_formation_control',
            executable='x_multi_state_publisher',
            name='x_multi_state_publisher',
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
