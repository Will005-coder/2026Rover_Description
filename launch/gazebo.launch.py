from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, AppendEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('urdf_description')

    set_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_path, '..')
    )

    xacro_file = os.path.join(pkg_path, 'urdf', 'URDF.xacro')
    robot_description = os.popen(f'xacro {xacro_file}').read()

    return LaunchDescription([

        set_resource_path,

        # Start Gazebo
        ExecuteProcess(cmd=['ign', 'gazebo', '-r', 'empty.sdf'], output='screen'),

        # publish robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
        ),

        # Spawn robot
        TimerAction(
            period=5.0,  
            actions=[
                Node(
                    package='ros_gz_sim', 
                    executable='create',
                    arguments=[
                        '-world', 'empty',
                        '-name', 'URDF',
                        '-topic', 'robot_description',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.5'
                    ],
                    output='screen'
                )
            ]
        )
                
    ])