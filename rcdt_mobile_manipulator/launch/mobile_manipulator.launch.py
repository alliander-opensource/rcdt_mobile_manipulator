# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import get_file_path, get_robot_description

xacro_file = get_file_path(
    "rcdt_mobile_manipulator", ["urdf"], "mobile_manipulator.urdf.xacro"
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

panther_controllers = IncludeLaunchDescription(
    get_file_path("rcdt_panther", ["launch"], "controllers.launch.py")
)

rviz = IncludeLaunchDescription(
    get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py"),
    launch_arguments={"rviz_frame": "odom"}.items(),
)

moveit = IncludeLaunchDescription(
    get_file_path("rcdt_utilities", ["launch"], "moveit.launch.py"),
    launch_arguments={
        "moveit": "servo",
        "moveit_config_package": "rcdt_mobile_manipulator_moveit_config",
    }.items(),
)

joy = Node(
    package="joy",
    executable="game_controller_node",
    parameters=[
        {"sticky_buttons": True},
    ],
)

joy_topic_manager = Node(
    package="rcdt_mobile_manipulator",
    executable="joy_topic_manager_node.py",
)

joy_to_twist_franka = Node(
    package="rcdt_utilities",
    executable="joy_to_twist_node.py",
    parameters=[
        {"sub_topic": "/franka/joy"},
        {"pub_topic": "/servo_node/delta_twist_cmds"},
        {"config_pkg": "rcdt_franka"},
    ],
)

joy_to_twist_panther = Node(
    package="rcdt_utilities",
    executable="joy_to_twist_node.py",
    parameters=[
        {"sub_topic": "/panther/joy"},
        {"pub_topic": "/diff_drive_controller/cmd_vel"},
        {"config_pkg": "rcdt_panther"},
    ],
)

joy_to_gripper = Node(
    package="rcdt_franka",
    executable="joy_to_gripper_node.py",
    parameters=[{"sub_topic": "/franka/joy"}],
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
            joy,
            joy_topic_manager,
            joy_to_twist_franka,
            joy_to_twist_panther,
            joy_to_gripper,
        ]
    )
