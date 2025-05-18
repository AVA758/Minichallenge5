import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess


from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the path to the URDF file
    
    # Declare launch arguments
    robot_arg = DeclareLaunchArgument('robot', default_value='puzzlebot_jetson_lidar_ed')   
    prefix_arg = DeclareLaunchArgument('prefix', default_value='')

    robot_name = LaunchConfiguration('robot')
    prefix_name = LaunchConfiguration('prefix')

    # Frame names - set your defaults or make arguments if you want dynamic
    lidar_frame_name = 'lidar_link'
    camera_frame_name = 'camera_link'
    tof_frame_name = 'tof_link'
    use_sim_time = True

    # Get package path for puzzlebot_description
    puzzlebot_description_dir = get_package_share_directory('puzzlebot_description')

    # Build robot_description param with xacro and parameters
    robot_description = Command([
        'xacro ',
        os.path.join(puzzlebot_description_dir, 'urdf', 'mcr2_robots') + '/',
        robot_name,
        '.xacro ',
        'prefix:=', prefix_name, ' ',
        'lidar_frame:=', lidar_frame_name, ' ',
        'camera_frame:=', camera_frame_name, ' ',
        'tof_frame:=', tof_frame_name,
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': use_sim_time,
        }],
        namespace=prefix_name,
    )



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
    puzzlebot_controller_node = Node(name='puzzlebot_controller',
                                    package='minichallenge4',
                                    executable='puzzlebot_controller',
                                    output='screen',
                                    parameters=[nodes_params_path, {'use_sim_time':True}],
                                    )
    
    #puzzlebot_sim_node = Node(name='puzzlebot_sim',
     #                           package='minichallenge4',
      #                          executable='puzzlebot_sim',
          #                      output='screen',
           #                     parameters=[global_params],
            #                    )
    
    puzzlebot_localization_node = Node(name='puzzlebot_localization',
                                    package='minichallenge4',
                                    executable='puzzlebot_localization',
                                    output='screen',
                                    parameters=[nodes_params_path, global_params],
                                    )
    
    puzzlebot_joint_publisher_node = Node(name='puzzlebot_joint_publisher',
                                            package='minichallenge4',
                                            executable='puzzlebot_joint_publisher',
                                            output='screen',
                                            parameters=[global_params],
                                           )
    
    
        
        
    puzzlebot_nodes.append(puzzlebot_controller_node)
    #puzzlebot_nodes.append(puzzlebot_sim_node)
    puzzlebot_nodes.append(puzzlebot_localization_node)
    puzzlebot_nodes.append(puzzlebot_joint_publisher_node)
  
        
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
    #rviz_node = Node(name='rviz2',
     #               package='rviz2',
      #              executable='rviz2',
       #             output='screen',
        #            arguments=['-d', rviz_config_path],)

    rviz_node = Node(name='rviz2',
                    package='rviz2',
                    executable='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config_path],)



    # Launch description
    l_d = LaunchDescription([robot_arg,
                            prefix_arg,
                            robot_state_publisher_node,
                            map_odom_transform_node,
                            *puzzlebot_nodes,
                            rqt_tf_tree_node,
                            rqt_graph_node,
                            rviz_node,])
    

    return l_d
