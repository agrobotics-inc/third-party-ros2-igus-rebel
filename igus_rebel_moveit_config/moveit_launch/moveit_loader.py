#!/usr/bin/env python3

# python imports
import os
import yaml

# ros2 imports
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    PathJoinSubstitution,
    TextSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ParameterValue


def load_yaml(package_name, file_path):
    """Load a yaml file from the specified package"""
    full_path = os.path.join(get_package_share_directory(package_name), file_path)
    try:
        with open(full_path, "r") as file:
            return yaml.safe_load(file)
    except (
            EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def declare_arguments():
    """ Returns list of launch arguments """
    rviz_file_arg = DeclareLaunchArgument(
        name="rviz_file",
        default_value="none",
        description="Path to the RViz configuration file",
    )

    hardware_protocol_arg = DeclareLaunchArgument(
        name="hardware_protocol",
        default_value="cri",
        choices=["mock_hardware", "cri", "simulation", "ignition"],
        description="Which hardware protocol or simulation environment should be used",
    )

    load_gazebo_arg = DeclareLaunchArgument(
        name="load_gazebo",
        default_value="false",
        choices=["true", "false"],
        description="Which Gazebo version to launch",
    )
    return [
        rviz_file_arg,
        hardware_protocol_arg,
        load_gazebo_arg,
    ]


def load_robot_description():
    """Load the robot description URDF"""

    robot_description_filename = "Rebel.urdf.xacro"

    robot_description_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_moveit_config"),
            "config",
            robot_description_filename,
        ]
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_file,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    return robot_description


def load_robot_description_semantic():
    """ Load robot semantic description SRDF file """

    robot_description_semantic_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_moveit_config"),
            "config",
            "Rebel.srdf",
        ]
    )

    robot_description_semantic_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_semantic_file,
        ]
    )

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }
    return robot_description_semantic


def load_ros2_controllers():
    """ Load ROS2 controllers yaml """

    ros2_controllers_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_moveit_config"),
            "config",
            "ros2_controllers.yaml",
        ]
    )
    return ros2_controllers_file


def load_moveit() -> list:
    """ Loads parameters required by move_group node interface """

    # OMPL planner
    ompl_planning_yaml = load_yaml(
        "igus_rebel_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config = {"move_group": {}}
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # STOMP planner
    stomp_planner_yaml = load_yaml(
        "igus_rebel_moveit_config", "config/stomp_planning.yaml"
    )
    stomp_planning_pipeline_config = {"move_group": {}}
    stomp_planning_pipeline_config["move_group"].update(stomp_planner_yaml)

    # Pilz cartesian limits
    pilz_cartesian_limits_yaml = load_yaml(
        "igus_rebel_moveit_config", "config/pilz_cartesian_limits.yaml"
    )
    pilz_cartesian_limits = {"robot_description_planning": pilz_cartesian_limits_yaml}
    
    # Multiple planners: STOMP and OMPL and Pilz industrial motion planner
    multiple_planners_yaml = load_yaml(
        "igus_rebel_moveit_config", "config/multiple_planning_pipelines.yaml"
    )
    planning_pipelines = {"move_group": multiple_planners_yaml}
    planning_pipelines["move_group"].update(pilz_cartesian_limits)

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "planning_plugin": "ompl_interface/OMPLPlanner",
    }

    kinematics_yaml = load_yaml(
        "igus_rebel_moveit_config", "config/kinematics.yaml")
    kinematics = {"robot_description_kinematics": kinematics_yaml}

    moveit_controllers_yaml = load_yaml(
        "igus_rebel_moveit_config", "config/moveit_controllers.yaml"
    )

    joint_limits_yaml = load_yaml(
        "igus_rebel_moveit_config", "config/joint_limits.yaml"
    )
    joint_limits = {"robot_description_planning": joint_limits_yaml}

    sensors_3d_yaml = {"sensors:": ""}

    return [
        load_robot_description(),
        load_robot_description_semantic(),
        #ompl_planning_pipeline_config, # single planner config
        #stomp_planning_pipeline_config, # single planner config
        multiple_planners_yaml, # multiple planners config
        pilz_cartesian_limits,
        planning_scene_monitor_parameters,
        kinematics,
        moveit_controllers_yaml,
        joint_limits,
        sensors_3d_yaml
    ]
