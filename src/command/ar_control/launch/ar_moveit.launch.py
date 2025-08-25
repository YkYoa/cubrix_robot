import os
import yaml
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import xacro

def loadFile(package_name, file_path):
    packagePath = get_package_share_directory(package_name)
    absoluteFilePath = os.path.join(packagePath, file_path)
    try:
        with open(absoluteFilePath, "r") as file:
            return file.read()
    except EnvironmentError:
        return None

def loadYaml(package_name, file_path):
    packagePath = get_package_share_directory(package_name)
    absoluteFilePath = os.path.join(packagePath, file_path)
    try:
        with open(absoluteFilePath, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def mergeDicts(dict1, dict2):
    merged = dict1.copy()
    for key, value in dict2.items():
        if key in merged:
            if isinstance(merged[key], dict) and isinstance(value, dict):
                merged[key] = mergeDicts(merged[key], value)
            else:
                merged[key] = value
        else:
            merged[key] = value
    return merged

def loadAndMergeYaml(package1_name, package2_name, file_path):
    yaml1 = loadYaml(package1_name, file_path)
    if package2_name == "":
        return yaml1
    yaml2 = loadYaml(package2_name, file_path)
    if yaml2 is None:
        return yaml1
    if yaml1 is None:
        return None
    return mergeDicts(yaml1, yaml2)

def generate_launch_description():
    # Parse 'desc' argument from command line
    descArg = ""
    package_version = ""
    package_project = ""
    delimiter = "_"

    for arg in sys.argv:
        if arg.startswith("desc:="):
            descArg = str(arg.split(":=")[1])
            if delimiter in descArg:
                package_version, package_project = descArg.split(delimiter, 1)
                descArg = descArg.replace(delimiter, "")
            else:
                package_version = descArg
                package_project = descArg

            # Ensure prefix 'ar' only once
            if not package_version.startswith("ar"):
                package_version = "ar" + package_version + "_moveit_config"
            else:
                package_version = package_version + "_moveit_config"

            if not package_project.startswith("ar"):
                package_project = "ar" + package_project + "_moveit_config"
            else:
                package_project = package_project + "_moveit_config"

    if descArg == "":
        print("ERROR: Robot description ('desc' argument) must be specified ********")
        sys.exit()

    rvizArgument = DeclareLaunchArgument("rviz", default_value="true", description="RViz on/off")
    rvizArg = LaunchConfiguration("rviz")

    # Semantic Description
    robotDescSemanticConfig = xacro.process_file(
        os.path.join(
            get_package_share_directory(package_project),
            "config",
            "ar_config.srdf.xacro",
        )
    )
    robotDescSemantic = {"robot_description_semantic": robotDescSemanticConfig.toxml()}

    # Kinematics, Joint and Cartesian Limits
    kinematicYaml = loadAndMergeYaml(package_version, package_project, "config/kinematics.yaml")
    robotDescKinematics = {"robot_description_kinematics": kinematicYaml}

    jointLimitsYaml = loadAndMergeYaml(package_version, package_project, "config/joint_limits.yaml")
    cartesianLimitsYaml = loadYaml(package_version, "config/cartesian_limits.yaml")
    robotDescPlanning = {
        "robot_description_planning": {
            **jointLimitsYaml,
            **cartesianLimitsYaml,
        }
    }

    # Planning Pipelines
    planningPipelinesConfig = {
        "default_planning_pipeline": "pilz",
        "planning_pipelines": ["pilz", "ompl"],
        "pilz": {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "request_adapters": "",
            "start_state_max_bounds_error": 0.1,
        },
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        },
    }

    omplPlanningYaml = loadAndMergeYaml(package_version, package_project, "config/ompl_planning.yaml")
    planningPipelinesConfig["ompl"].update(omplPlanningYaml)

    # MoveIt Controllers
    moveitSimpleControllersYaml = loadYaml(package_project, "config/ar_moveit_controllers.yaml")
    moveitControllers = {
        "moveit_simple_controller_manager": moveitSimpleControllersYaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # Trajectory Execution
    trajectoryExecution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planningSceneMonitorParm = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    # Start move_group node
    runMoveGroupNode = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        emulate_tty=True,
        parameters=[
            robotDescSemantic,
            robotDescKinematics,
            robotDescPlanning,
            planningPipelinesConfig,
            trajectoryExecution,
            moveitControllers,
            planningSceneMonitorParm,
            jointLimitsYaml,
        ],
    )

    # Optional: RViz Launch (disabled by default)
    rvizLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/ar_rviz.launch.py']),
        launch_arguments={"desc": descArg}.items(),
        condition=IfCondition(rvizArg)
    )

    return LaunchDescription([
        runMoveGroupNode,
        # rvizArgument,
        # rvizLaunch,
    ])
