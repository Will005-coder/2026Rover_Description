from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument 
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory 
import os
import xacro
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    # path definitions
    pkg_path = get_package_share_directory("urdf_description")
    
    default_model_path = os.path.join(pkg_path, 'urdf', 'URDF.xacro')
    default_rviz_path = os.path.join(pkg_path, 'rviz', 'urdf.rviz')

    robot_description_content = ParameterValue(
        Command(['xacro ', default_model_path]),
        value_type=str
    )

    # model arguements
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path, 
        description='Path to robot xacro file'
    )

    # gui arguments
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Use joint_state_publisher_gui'
    )

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_path,
        description='rviz config file'
    )

    # rsp node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content
        }]
    )

    # jsp node
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # rviz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen'
    )


    return LaunchDescription([
        model_arg,
        gui_arg,
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
