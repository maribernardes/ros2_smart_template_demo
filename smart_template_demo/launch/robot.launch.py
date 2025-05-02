from launch import LaunchDescription
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


# Launch SmartTemplate demo (virtual_template node and robot_description)
# Includes world_pose_listener node (for adjusting robot in the world coordinates)
# Includes optional for GUI to command with robot

def generate_launch_description():

    ld = LaunchDescription()

    arg_rviz = DeclareLaunchArgument(
        'rviz', 
        default_value = 'false', 
        choices = ['true', 'false'],
        description = 'Start RViz automatically'
    )

    arg_gui = DeclareLaunchArgument(
        'gui', 
        default_value = 'true', 
        choices = ['true', 'false'],
        description = 'Start SmartTemplate GUI plugin automatically'
    )

    arg_description_package = DeclareLaunchArgument(
        'description_package',
        default_value = 'smart_template_description',
        description = 'Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.',
    )

    arg_description_file = DeclareLaunchArgument(
        'description_file',
        default_value = 'smart_template.urdf.xacro',
        description = 'URDF/XACRO description file with the robot'
    )
    
    arg_name = DeclareLaunchArgument(
        'name',
        default_value = 'smart_template',
        description = 'Name of the robot system'
    )
    
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    rviz_file = PathJoinSubstitution([FindPackageShare(description_package), 'rviz', 'urdf.rviz'])
    rate = LaunchConfiguration('rate', default=50.0)  # Hz, default is 10 so we're increasing that a bit. 
    # Funny enough joint and robot state publishers don't have the same name for that parameter :-(
    
    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), 'urdf', description_file]),
        " ",
        "name:=", LaunchConfiguration('name')
    ])
    robot_description = {
        "robot_description": robot_description_content, 'publish_frequency': rate
    }
    
    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        arguments=[description_file]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
        name='rviz2',
        output='screen'
    )

    virtual_robot_node = Node(
        package="smart_template_demo",
        executable="virtual_template",
        parameters=[robot_description]
    )  
    
    world_pose_node = Node(
        package='smart_template_demo',
        executable='world_pose_listener',
        name='world_pose_listener',
        output='screen',
    )

    # Event handler to launch the GUI plugin after the robot_state_publisher is started
    gui_plugin_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[
                ExecuteProcess(
                    condition=IfCondition(LaunchConfiguration('gui')),
                    cmd=['rqt', '--standalone', 'smart_template_gui', '--force-discover'],
                    output='screen'
                )
            ]
        )
    )

    # Include launch arguments
    ld.add_action(arg_rviz)
    ld.add_action(arg_gui)
    ld.add_action(arg_description_package)
    ld.add_action(arg_description_file)
    ld.add_action(arg_name)
    
    #Nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(virtual_robot_node)
    ld.add_action(world_pose_node)
    ld.add_action(gui_plugin_event_handler)

    return ld
