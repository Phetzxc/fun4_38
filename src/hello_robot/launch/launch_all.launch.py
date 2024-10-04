from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro    
    
def generate_launch_description():
    
    pkg = get_package_share_directory('example_description')
    
    # Assuming you want to use the uploaded rviz.rviz file
    rviz_path = os.path.join(pkg, 'config', 'rviz.rviz')
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen'
    )
    
    path_description = os.path.join(pkg,'robot','visual','my-robot.xacro')
    robot_desc_xml = xacro.process_file(path_description).toxml()
    
    parameters = [{'robot_description': robot_desc_xml}]
    
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=parameters
    )
    
    jointstate_node = Node(
        package='example_description',  # Replace this with the correct package name if different
        executable='jointstate_script.py',  # Ensure this is executable
        output='screen'
    )
    
    plot_ws_node = Node(
        package='example_description',  # Replace this with the correct package name
        executable='plot_ws.py',  # Ensure this is executable
        output='screen'
    )
    
    random_target_node = Node(
        package='example_description',  # Replace with the correct package name
        executable='random_target.py',  # Ensure this is executable
        output='screen'
    )
    
    teleop_node = Node(
        package='example_description',  # Replace this with the correct package name
        executable='teleop.py',  # Ensure this is executable
        output='screen'
    )
    
    launch_description = LaunchDescription()
    
    launch_description.add_action(rviz)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(jointstate_node)
    launch_description.add_action(plot_ws_node)
    launch_description.add_action(random_target_node)
    launch_description.add_action(teleop_node)
    
    return launch_description
