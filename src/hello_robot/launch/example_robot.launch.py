#!/usr/bin/env python3

"""
This program is free software: you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.

created by Thanacha Choopojcharoen at CoXsys Robotics (2022)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro    

def generate_launch_description():
    
    # Package directory
    pkg = get_package_share_directory('example_description')

    # RViz configuration file
    rviz_path = os.path.join(pkg, 'config', 'display.rviz')

    # Create RViz Node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen'
    )

    # Load robot description from the xacro file
    path_description = os.path.join(pkg, 'robot', 'visual', 'my-robot.xacro')
    robot_desc_xml = xacro.process_file(path_description).toxml()
    
    # Parameters for the robot description
    parameters = [{'robot_description': robot_desc_xml}]
    
    # Robot State Publisher Node (optional, uncomment if needed)
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=parameters
    # )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )
    
    # Nodes for your custom scripts
    jointstate_node = Node(
        package='example_description',  # Adjust this if the package name is different
        executable='jointstate_script.py',  # Make sure the script is executable
        output='screen'
    )
    
    plot_ws_node = Node(
        package='example_description',
        executable='plot_ws.py',
        output='screen'
    )
    
    random_target_node = Node(
        package='example_description',
        executable='random_target.py',
        output='screen'
    )
    
    teleop_node = Node(
        package='example_description',
        executable='teleop.py',
        output='screen'
    )
    
    # Launch description
    launch_description = LaunchDescription()

    # Add actions to the launch description
    launch_description.add_action(rviz)
    # launch_description.add_action(robot_state_publisher)  # Uncommented for full functionality
    launch_description.add_action(joint_state_publisher)
    launch_description.add_action(jointstate_node)
    launch_description.add_action(plot_ws_node)
    launch_description.add_action(random_target_node)
    launch_description.add_action(teleop_node)
    
    return launch_description
