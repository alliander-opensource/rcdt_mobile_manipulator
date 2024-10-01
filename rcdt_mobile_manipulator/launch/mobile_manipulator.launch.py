# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import get_file_path, get_robot_description

# Get robot_description:
xacro_file = get_file_path(
    "rcdt_mobile_manipulator", ["config"], "mobile_manipulator.urdf.xacro"
)
robot_description = get_robot_description(xacro_file)

robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[robot_description],
)

gazebo_robot = IncludeLaunchDescription(
    get_file_path("rcdt_utilities", ["launch"], "gazebo_robot.launch.py")
)

fr3_gripper = Node(
    package="rcdt_franka",
    executable="simulated_gripper_node.py",
    output="screen",
)

controllers_config = get_file_path(
    "rcdt_franka", ["config"], "simulation_controllers.yaml"
)
gripper_action_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["gripper_action_controller", "-p", controllers_config],
)

fr3_arm_controller_config = get_file_path(
    "rcdt_franka", ["config"], "simulation_controllers.yaml"
)
fr3_arm_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["fr3_arm_controller", "-p", fr3_arm_controller_config],
)

rviz = IncludeLaunchDescription(
    get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py"),
    launch_arguments={"rviz_frame": "base_link"}.items(),
)

joint_state_broadcaster = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "joint_state_broadcaster",
        "-t",
        "joint_state_broadcaster/JointStateBroadcaster",
    ],
)

moveit = IncludeLaunchDescription(
    get_file_path("rcdt_utilities", ["launch"], "moveit.launch.py"),
    launch_arguments={
        "moveit": "servo",
        "moveit_config_package": "rcdt_mobile_manipulator_moveit_config",
    }.items(),
)

# get controller parameters:
controllers_config = get_file_path(
    "rcdt_mobile_manipulator", ["config"], "ros_controller.yaml"
)

# diff drive controller:
diff_drive_controller = Node(
    package="controller_manager",
    executable="spawner",
    name="diff_drive_controller",
    arguments=[
        "diff_drive_controller",
        "-t",
        "diff_drive_controller/DiffDriveController",
        "-p",
        controllers_config,
    ],
)


def generate_launch_description() -> None:
    return LaunchDescription(
        [
            robot_state_publisher,
            gazebo_robot,
            joint_state_broadcaster,
            fr3_arm_controller,
            fr3_gripper,
            gripper_action_controller,
            moveit,
            rviz,
        ]
    )
