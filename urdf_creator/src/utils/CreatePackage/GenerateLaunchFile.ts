export function GenerateRealLaunchFile(title: string) {
    return `
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the package share directory
    package_share_directory = get_package_share_directory("${title}_description")

    # Define the path to the URDF and RViz files
    urdf_file = os.path.join(package_share_directory, "urdf", "${title}.urdf")

    # Define the robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": open(urdf_file).read()}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            robot_state_publisher_node,
        ]
    )
`;
}

export function GenerateVisualLaunchFile(title: string) {
    return `
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the package share directory
    package_share_directory = get_package_share_directory("${title}_description")

    # Define the path to the URDF and RViz files
    urdf_file = os.path.join(package_share_directory, "urdf", "${title}.urdf")
    rviz_file = os.path.join(package_share_directory, "rviz", "my_robot.rviz")

    # Define the robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": open(urdf_file).read()}],
    )

    # Define the RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            robot_state_publisher_node,
            rviz_node,
        ]
    )
`;
}

export function GenerateTestLaunchFile(title: string) {
    return `
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the package share directory
    package_share_directory = get_package_share_directory("${title}_description")

    # Define the path to the URDF and RViz files
    urdf_file = os.path.join(package_share_directory, "urdf", "${title}.urdf")
    rviz_file = os.path.join(package_share_directory, "rviz", "my_robot.rviz")

    # Define the robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": open(urdf_file).read()}],
    )

    joint_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # Define the RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            robot_state_publisher_node,
            rviz_node,
            joint_gui,
        ]
    )
`;
}

export function GenerateSimLaunchFile(title: string, world: string) {
    // Define the package share directories and file paths section
    const packagePaths = `
    # Get the package share directory
    package_share_directory = get_package_share_directory("${title}_description")
    simulation_package = get_package_share_directory("${title}_gazebo")

    # Define the path to the URDF, RViz, and SDF files
    urdf_file = os.path.join(package_share_directory, "urdf", "${title}.urdf")
    rviz_file = os.path.join(package_share_directory, "rviz", "my_robot.rviz")
    gazebo_launch_file = os.path.join(
        get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
    )
    sdf_file = os.path.join(simulation_package, "model", "${title}.sdf")
`;

    const worldFile = `world = "${world}" 
    world_file = os.path.join(simulation_package, "worlds", world)
`;

    // Define the robot_state_publisher node section
    const robotStatePublisherNode = `
    # Define the robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": open(urdf_file).read()}],
    )
`;

    // Define the RViz node section
    const rvizNode = `
    # Define the RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
    )
`;

    // Define the Gazebo node section with dynamic world file
    let gazeboNode = `
    # Define the Gazebo node
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={"world": world_file}.items(),
    )
`;

    if (world === "empty.world") {
        gazeboNode = `
    # Define the Gazebo node
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={"world": "empty.world"}.items(),
    )
`;
    }

    // Define the spawn_entity node section
    const spawnEntityNode = `
    # Define the spawn_entity node to spawn the robot in Gazebo
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",
            sdf_file,
            "-entity",
            "${title}",
        ],
        output="screen",
    )
`;

    // Combine all sections and return the LaunchDescription
    return `
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ${packagePaths}
    ${robotStatePublisherNode}
    ${rvizNode}
    ${worldFile}
    ${gazeboNode}
    ${spawnEntityNode}

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            robot_state_publisher_node,
            rviz_node,
            gazebo_node,
            spawn_entity_node,
        ]
    )
`;
}
