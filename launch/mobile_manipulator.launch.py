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

# gazebo:
gazebo = IncludeLaunchDescription(
    get_file_path("ros_gz_sim", ["launch"], "gz_sim.launch.py"),
    launch_arguments={
        "gz_args": "empty.sdf -r",
    }.items(),
)

# robot_state_publisher:
robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    output="both",
    parameters=[robot_description],
)

# spawn robot:
spawn_robot = Node(
    package="ros_gz_sim",
    executable="create",
    arguments=["-topic", "/robot_description"],
    output="screen",
)

# get controller parameters:
controller_parameters = get_file_path(
    "rcdt_mobile_manipulator", ["config"], "ros_controller.yaml"
)

# joint_trajectory_controller:
joint_trajectory_controller = Node(
    package="controller_manager",
    executable="spawner",
    name="joint_trajectory_controller",
    arguments=[
        "joint_trajectory_controller",
        "-t",
        "joint_trajectory_controller/JointTrajectoryController",
        "-p",
        controller_parameters,
    ],
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
        controller_parameters,
    ],
)

# clock sync, otherwise diff_drive_conroller doesn't work:
clock_sync = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    name="panther_base_gz_bridge",
    arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
)


def generate_launch_description() -> None:
    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            spawn_robot,
            joint_trajectory_controller,
            diff_drive_controller,
            clock_sync,
        ]
    )
