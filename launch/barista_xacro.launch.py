import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix

import xacro


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )

    robot_model_path = os.path.join(
        get_package_share_directory('barista_robot_description'))

    xacro_file = os.path.join(robot_model_path, 'xacro', 'barista_robot_model.urdf.xacro')

    # convert XACRO file into URDF
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'barista_robot', '-x', '0.0', '-y', '0.0', '-z', '0.1107',
                                   '-topic', 'robot_description'],
                        output='screen')

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory('barista_robot_description'), 'rviz', 'urdf_vis.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    return LaunchDescription([
        gazebo,
        rviz_node,
        robot_state_publisher,
        spawn_entity,
    ])