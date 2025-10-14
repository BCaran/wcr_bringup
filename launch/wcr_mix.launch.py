from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess

def generate_launch_description():

    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare("wcr_description"),
            "urdf",
            "wcr_mix.urdf.xacro",
        ]
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("wcr_bringup"),
            "config",
            "wcr_controllers_mix.yaml",
        ]
    )

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    driving_effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["driving_effort_controller", "--controller-manager", "/controller_manager"],
    )

    steering_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steering_position_controller", "--controller-manager", "/controller_manager"],
    )
    
    init_command = ExecuteProcess(
    cmd=[
        "ros2", "topic", "pub", "--once",
        "/steering_position_controller/commands",
        "std_msgs/msg/Float64MultiArray",
        "{data: [-0.785398163, 0.785398163, -0.785398163, 0.785398163]}"
    ],
    output="screen"
)

    return LaunchDescription([
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        driving_effort_controller_spawner,
        steering_position_controller,
        init_command
    ])