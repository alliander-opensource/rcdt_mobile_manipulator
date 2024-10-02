# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import get_file_path, get_robot_description

xacro_file = get_file_path(
    "rcdt_mobile_manipulator", ["config"], "mobile_manipulator.urdf.xacro"
)
robot_description = get_robot_description(xacro_file)

robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[robot_description],
)

robot = IncludeLaunchDescription(
    get_file_path("rcdt_utilities", ["launch"], "gazebo_robot.launch.py")
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

franka_controllers = IncludeLaunchDescription(
    get_file_path("rcdt_franka", ["launch"], "controllers.launch.py"),
    launch_arguments={
        "simulation": "True",
        "arm_controller": "fr3_arm_controller",
        "gripper_controller": "fr3_gripper",
    }.items(),
)

# TODO: Move panther controllers to panther repo.
panther_controllers = IncludeLaunchDescription(
    get_file_path("rcdt_panther", ["launch"], "controllers.launch.py")
)

rviz = IncludeLaunchDescription(
    get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py"),
    launch_arguments={"rviz_frame": "base_link"}.items(),
)

moveit = IncludeLaunchDescription(
    get_file_path("rcdt_utilities", ["launch"], "moveit.launch.py"),
    launch_arguments={
        "moveit": "servo",
        "moveit_config_package": "rcdt_mobile_manipulator_moveit_config",
    }.items(),
)


def generate_launch_description() -> None:
    return LaunchDescription(
        [
            robot_state_publisher,
            robot,
            joint_state_broadcaster,
            franka_controllers,
            panther_controllers,
            rviz,
            moveit,
        ]
    )
