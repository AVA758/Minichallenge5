import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the URDF file
    urdf_filename = 'puzzlebot.urdf'
    urdf_path = os.path.join(get_package_share_directory('minichallenge4'), 'urdf', urdf_filename)
    # Read the URDF file
    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    # Get the nodes parameters file
    nodes_params_filename = 'puzzlebot_nodes_params.yaml'
    nodes_params_path = os.path.join(get_package_share_directory('minichallenge4'), 'config', nodes_params_filename)

    # Get the global parameters file
    global_params_filename = 'puzzlebot_global_params.yaml'
    global_params_path = os.path.join(get_package_share_directory('minichallenge4'), 'config', global_params_filename)
    # Read the global parameters file
    with open(global_params_path, 'r') as file:
        global_params = yaml.safe_load(file)

    # Get the rviz config file
    rviz_config_filename = 'puzzlebot_view.rviz'
    rviz_config_path = os.path.join(get_package_share_directory('minichallenge4'), 'rviz', rviz_config_filename)

    # Get the robots declared in the config file
    with open(nodes_params_path, 'r') as file:
        nodes_params = yaml.safe_load(file)

    # Map to odom transform node
    map_odom_transform_node = Node(name='map_odom_transform',
                                    package='tf2_ros',
                                    executable='static_transform_publisher',
                                    output='screen',
                                    arguments=['--x', '1', '--y', '1', '--z', '0', 
                                                '--yaw', '0', '--pitch', '0', '--roll', '0', 
                                                '--frame-id', 'map', '--child-frame-id', 'odom'],)

    # Iterative puzzlebot nodes
    puzzlebot_nodes = []
    for puzzlebot_namespace in nodes_params.keys():
        puzzlebot_controller_node = Node(name='puzzlebot_controller',
                                        package='minichallenge4',
                                        executable='puzzlebot_controller',
                                        output='screen',
                                        parameters=[nodes_params_path],
                                        namespace=puzzlebot_namespace,
                                        remappings=[(f'/{puzzlebot_namespace}/cmd_vel', '/cmd_vel')],
                                        )
        
        puzzlebot_localization_node = Node(name='puzzlebot_localization',
                                        package='minichallenge4',
                                        executable='puzzlebot_localization',
                                        output='screen',
                                        parameters=[nodes_params_path, global_params],
                                        namespace=puzzlebot_namespace,
                                        remappings=[(f'/{puzzlebot_namespace}/wr', '/VelocityEncR'),
                                                    (f'/{puzzlebot_namespace}/wl', '/VelocityEncL'),],)
        
        puzzlebot_joint_publisher_node = Node(name='puzzlebot_joint_publisher',
                                                package='minichallenge4',
                                                executable='puzzlebot_joint_publisher',
                                                output='screen',
                                                parameters=[global_params],
                                                namespace=puzzlebot_namespace,)
        
        robot_description_node = Node(name='robot_state_publisher',
                                    package='robot_state_publisher',
                                    executable='robot_state_publisher',
                                    output='screen',
                                    parameters=[{'frame_prefix': f'{puzzlebot_namespace}/',
                                                'robot_description': robot_description}],
                                    namespace=puzzlebot_namespace,)
        
        puzzlebot_nodes.append(puzzlebot_controller_node)
        puzzlebot_nodes.append(puzzlebot_localization_node)
        puzzlebot_nodes.append(puzzlebot_joint_publisher_node)
        puzzlebot_nodes.append(robot_description_node)
        
    # RQT nodes
    rqt_tf_tree_node = Node(name='rqt_tf_tree',
                        package='rqt_tf_tree',
                        executable='rqt_tf_tree',
                        output='screen',)
    
    rqt_graph_node = Node(name='rqt_graph',
                    package='rqt_graph',
                    executable='rqt_graph',
                    output='screen',)
    
    # RViz node
    rviz_node = Node(name='rviz2',
                    package='rviz2',
                    executable='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config_path],)

    # Launch description
    l_d = LaunchDescription([map_odom_transform_node,
                            *puzzlebot_nodes,
                            rqt_tf_tree_node,
                            rqt_graph_node,
                            rviz_node,])
    

    return l_d
