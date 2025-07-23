import os
import yaml
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
import xacro

def loadYaml(package_name, file_path):
	packagePath = get_package_share_directory(package_name)
	absoluteFilePath = os.path.join(packagePath, file_path)

	try:
		with open(absoluteFilePath, "r") as file:
			return yaml.safe_load(file)
	except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
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

	# Command-line arguments
	# descArg = ""
	delimiter = "_"
	package_version = ""
	# package_project = ""

	package_version = "ar_moveit_config"
	# package_project = "ar_moveit_config"

	# for arg in sys.argv:
	# 	if arg.startswith("desc:="):
	# 		descArg = str(arg.split(":=")[1])
	# 		if delimiter in descArg:
	# 			package_version, package_project = descArg.split(delimiter, 1)
	# 			descArg = descArg.replace(delimiter, "")
	# 			package_version = "ar" + package_version + "_moveit_config"
	# 			package_project = "ar" + descArg + "_moveit_config"
	# 		else:
	# 			package_version = "ar" + descArg + "_moveit_config"

	# if descArg == "":
	# 	print("ERROR: Robot description ('desc' argument) must be specified ********")
	# 	sys.exit()

	simArgument = DeclareLaunchArgument("sim", default_value="false", description="Simulation mode on/off")
	simArg = LaunchConfiguration("sim")
	uiArgument = DeclareLaunchArgument("ui", default_value="false", description="Ui mode on/off")
	uiArg = LaunchConfiguration("ui")


	delayPeriod = 0.0 if (IfCondition(simArg)) else 8.0

	xacroPath =	os.path.join(get_package_share_directory("ar_moveit_config"),
			"config", "ar_config.urdf.xacro")
	robotDesc = {"robot_description": Command(['xacro ', xacroPath, ' sim:=', simArg, ' ui:=', uiArg])}

	robot_description_semantic_config = xacro.process_file(
		os.path.join(
			get_package_share_directory("ar_moveit_config"),
			"config",
			"ar_config.srdf.xacro",
		)
	)
	robot_description_semantic = {
		"robot_description_semantic": robot_description_semantic_config.toxml()
	}

	kinematics_yaml = loadYaml(package_version, "config/kinematics.yaml")
	robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

	# cpu_affinity = {"cpu_affinity": 4}

	# Static TF
	staticTf = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		name="static_transform_publisher",
		output="log",
		emulate_tty=True,
		arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "Body_base"],
	)

	# Publish TF
	robotStatePublisher = Node(
		package="robot_state_publisher",
		executable="robot_state_publisher",
		name="robot_state_publisher",
		output="both",
		emulate_tty=True,
		parameters=[robotDesc],
	)

	# ros2_control using arSystem as hardware
	ros2ControllersPath = os.path.join(
		get_package_share_directory("ar_moveit_config"),
		"config",
		"ar_ros_controllers.yaml",
	)
	ros2ControlNode = Node(
		package="controller_manager",
		executable="ros2_control_node",
		parameters=[
			robotDesc,
			robot_description_semantic,
			robot_description_kinematics,
			ros2ControllersPath],
		emulate_tty=True,
		output={
			"stdout": "screen",
			"stderr": "screen",
		},
	)

	moveitSimpleControllersYaml = loadYaml("ar_moveit_config", "config/ar_moveit_controllers.yaml")
	# Load controllers
	loadControllers = []
	for controller in moveitSimpleControllersYaml["controller_names"]:
		loadControllers += [
			ExecuteProcess(
				cmd=["ros2 run controller_manager spawner {}".format(controller)],
				shell=True,
				output="screen",
			)
		]

	jointStateBroadcasterSpawner = Node(
		package="controller_manager",
		executable="spawner",
		emulate_tty=True,
		arguments=[
			"joint_state_broadcaster",
			"--controller-manager",
			"/controller_manager",
			"--controller-manager-timeout",
            "300",
		],
	)

	delayJointStateBroadcasterSpawnerAfterRos2ControlNode = (
		RegisterEventHandler(
			event_handler=OnProcessStart(
				target_action=ros2ControlNode,
				on_start=[
					TimerAction(
						period = delayPeriod,
						actions = [jointStateBroadcasterSpawner],
					),
				],
			)
		)
	)

	delayRobotControllerSpawnersAfterJointStateBroadcasterSpawner = []
	spawner_delay = 0
	for controller in loadControllers:
		spawner_delay+=2.0
		delayRobotControllerSpawnersAfterJointStateBroadcasterSpawner += [
			RegisterEventHandler(
				event_handler=OnProcessExit(
					target_action=jointStateBroadcasterSpawner,
					on_exit=[
						TimerAction(
							period = spawner_delay,
							actions = [controller],
						),
					],
				)
			)
		]

	return LaunchDescription(
		[
			simArgument,
			uiArgument,
			staticTf,
			robotStatePublisher,
			ros2ControlNode,
			delayJointStateBroadcasterSpawnerAfterRos2ControlNode,
			Node(
				package='ar_control',
				executable='ar_control_server',
				name='ar_control_server',
				output='screen',
				parameters=[],
			),
		]
		+ delayRobotControllerSpawnersAfterJointStateBroadcasterSpawner
	)