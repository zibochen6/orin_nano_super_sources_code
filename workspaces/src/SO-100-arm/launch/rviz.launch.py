from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os

def get_robot_description(context, *args, **kwargs):
    dof = LaunchConfiguration('dof').perform(context)
    pkg_share = FindPackageShare('so_100_arm').find('so_100_arm')
    urdf_path = os.path.join(pkg_share, 'urdf', f'so_100_arm_{dof}dof.urdf')
    rviz_path = os.path.join(pkg_share, 'config', 'urdf.rviz')
    
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )
    
    return {'urdf_path': urdf_path, 'rviz_path': rviz_path, 'robot_description': robot_description}

def generate_launch_description():
    # Declare the DOF argument
    dof_arg = DeclareLaunchArgument(
        'dof',
        default_value='5',
        description='DOF configuration - either 5 or 7'
    )

    def launch_setup(context, *args, **kwargs):
        params = get_robot_description(context)
        
        robot_state_pub = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': params['robot_description']}]
        )

        joint_state_pub = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )

        nodes = [
            robot_state_pub,
            joint_state_pub,
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', params['rviz_path']]
            )
        ]
        return nodes

    return LaunchDescription([
        dof_arg,
        OpaqueFunction(function=launch_setup)
    ]) 