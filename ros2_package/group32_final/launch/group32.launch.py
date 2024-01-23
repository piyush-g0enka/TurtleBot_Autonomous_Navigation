import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # find the parameter file
    parameter_file = os.path.join(
        get_package_share_directory('group32_final'),
        'config',
        'waypoint_params.yaml')
    # Print the parameter file path
    print("Using parameter file:", parameter_file)

    # create a node to publish the parts tf
    parts_tf_node = Node(
        package='group32_final',
        executable="parts_broadcaster",
        name='Parts_TF_Broadcaster',
        output='screen'
    )

    # create a node to control the robot and use the parameter file
    waypoint_node = Node(
        package='group32_final',
        executable="wp_publisher",
        name='wp_publisher',
        parameters=[parameter_file],
        output='screen'
    )
    
    robot_move_node = Node(
        package='group32_final',
        executable="robot_move",
        name='navigate_through_poses_client',
        output='screen'
    )

    ld = LaunchDescription()

    # add the nodes to the launch description
    ld.add_action(parts_tf_node)
    ld.add_action(waypoint_node)
    ld.add_action(robot_move_node)

    return ld
