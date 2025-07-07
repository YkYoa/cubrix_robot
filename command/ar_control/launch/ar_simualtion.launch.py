import os
import yaml
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def loadFile(package_name, file_path):
	packagePath = get_package_share_directory(package_name)
	absoluteFilePath = os.path.join(packagePath, file_path)

	try:
		with open(absoluteFilePath, "r") as file:
			return file.read()
	except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
		return None


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
	# package_version = ""
	# package_project = ""
	# delimiter = "_"
    package_version = "ar_moveit_config"

	# for arg in sys.argv:
	# 	if arg.startswith("desc:="):
	# 		descArg = str(arg.split(":=")[1])
	# 		if delimiter in descArg:
	# 			package_version, package_project = descArg.split(delimiter, 1)
	# 			descArg = descArg.replace(delimiter, "")
	# 			package_version = "ar" + package_version + "_moveit_config"
	# 			package_project = "ar_moveit_config"
	# 		else:
	# 			package_version = "ar_moveit_config"

	if descArg == "":
		print("ERROR: Robot description ('desc' argument) must be specified ********")
		sys.exit()

	rviz_dir = os.path.join(
			get_package_share_directory("ar_control"),
			"launch",
			"moveit_empty.rviz",
		)

	robot_description_config = xacro.process_file(
		os.path.join(
			get_package_share_directory("ar_moveit_config"),
			"config",
			"ar_config.urdf.xacro",
		)
	)
	robot_description = {"robot_description": robot_description_config.toxml()}

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

	# RViz
	return LaunchDescription(
		[
			Node(
				package="rviz2",
				executable="rviz2",
				output="log",
				emulate_tty=True,
				arguments=["-d", rviz_dir, "--display-title-format", "ar_Rviz"],
				parameters=[
					robot_description,
					robot_description_semantic,
                    robot_description_kinematics,
				]
			)
		]
	)
