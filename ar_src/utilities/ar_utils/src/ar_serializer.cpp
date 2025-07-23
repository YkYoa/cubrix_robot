/// Serialization functions collection; supports reading of different data structures from yaml files
/// Udupa; Apr'22
/// Added data caching option for optimization; Udupa; Jul'22
/// Added write data functions; Udupa; Jul'22

#include <boost/algorithm/string.hpp>
#include <fstream>
#include <stdio.h>
#include <string>
#include <vector>

#include "ar_serializer.h"

#include "ar_utils.h"
auto SERIAL_LOGGER = rclcpp::get_logger("ar_serializer");

namespace ar_utils
{
	YAML::Node pose_data_sm;
	std::string pose_path_sm;
	bool cache_data = false;
	std::map<std::string, YAML::Node> file_data;

	void clearFileCache(const std::string& filePath)
	{
		if(filePath.empty()) {
			file_data.clear();
			printf(COLOR_GRAY "    Cleared all file cache\n" COLOR_RESET);
		}
		else {
			file_data.erase(filePath);
			printf(COLOR_GRAY "    Cleared file cache %s\n" COLOR_RESET, filePath.c_str());
		}
		fflush(stdout);
	}

	/// Read file sequence one time to fetch goal or something from file quickly
	void readFileSm(std::string pose_path)
	{
		pose_data_sm = YAML::LoadFile(pose_path);
		pose_path_sm = pose_path;
	}

	void readFile(const std::string& filePath, YAML::Node& fileData)
	{
		if(cache_data) {
			if(file_data.find(filePath) == file_data.end())
				file_data[filePath] = YAML::LoadFile(filePath);
			fileData = file_data[filePath];
		}
		else {
			fileData = YAML::LoadFile(filePath);
		}
	}

	void writeToFile(const std::string& filePath, YAML::Node& fileData)
	{
		std::ofstream fileStream(filePath);
		fileStream << fileData;

		if(cache_data)
			file_data[filePath] = fileData;
	}

	std::string readString(const std::string& posePath, const std::string& dataSection, const std::string& key)
	{
		try {
			std::string str;
			if(posePath == pose_path_sm) {
				str = pose_data_sm[dataSection][key].as<std::string>();
				printf(COLOR_GRAY "    Fetched %s.%s: %s\n" COLOR_RESET, dataSection.c_str(), key.c_str(), str.c_str());
			}
			else {
				YAML::Node poseData;
				readFile(posePath, poseData);
				str = poseData[dataSection][key].as<std::string>();
				printf(COLOR_PURPLE "    Fetched %s.%s: %s\n" COLOR_RESET, dataSection.c_str(), key.c_str(), str.c_str());
			}
			fflush(stdout);
			return str;
		}
		catch(...) {
			RCLCPP_ERROR(SERIAL_LOGGER, "Failed to read %s.%s from file %s\n", dataSection.c_str(), key.c_str(), posePath.c_str());
			return "";
		}
	}

	bool writeString(const std::string& posePath, const std::string& dataSection, const std::string& key, const std::string& data)
	{
		try {
			if(posePath == pose_path_sm) {
				pose_data_sm[dataSection][key] = data;
				writeToFile(posePath, pose_data_sm);
				printf(COLOR_GRAY "    Wrote %s.%s: %s\n" COLOR_RESET, dataSection.c_str(), key.c_str(), data.c_str());
			}
			else {
				YAML::Node poseData;
				readFile(posePath, poseData);
				poseData[dataSection][key] = data;
				writeToFile(posePath, poseData);
				printf(COLOR_PURPLE "    Wrote %s.%s: %s\n" COLOR_RESET, dataSection.c_str(), key.c_str(), data.c_str());
			}
			fflush(stdout);
		}
		catch(...) {
			RCLCPP_ERROR(SERIAL_LOGGER, "Failed to write to %s.%s in file %s\n", dataSection.c_str(), key.c_str(), posePath.c_str());
			return false;
		}

		return true;
	}

	int readInt(const std::string& posePath, const std::string& dataSection, const std::string& key)
	{
		try {
			int val;
			if(posePath == pose_path_sm) {
				val = pose_data_sm[dataSection][key].as<int>();
				printf(COLOR_GRAY "    Fetched %s.%s: %d\n" COLOR_RESET, dataSection.c_str(), key.c_str(), val);
			}
			else {
				YAML::Node poseData;
				readFile(posePath, poseData);
				val = poseData[dataSection][key].as<int>();
				printf(COLOR_PURPLE "    Fetched %s.%s: %d\n" COLOR_RESET, dataSection.c_str(), key.c_str(), val);
			}
			fflush(stdout);
			return val;
		}
		catch(...) {
			RCLCPP_ERROR(SERIAL_LOGGER, "Failed to read %s.%s from file %s\n", dataSection.c_str(), key.c_str(), posePath.c_str());
			return INT_MAX;
		}
	}

	bool writeInt(const std::string& posePath, const std::string& dataSection, const std::string& key, const int& data)
	{
		try {
			if(posePath == pose_path_sm) {
				pose_data_sm[dataSection][key] = data;
				writeToFile(posePath, pose_data_sm);
				printf(COLOR_GRAY "    Wrote %s.%s: %d\n" COLOR_RESET, dataSection.c_str(), key.c_str(), data);
			}
			else {
				YAML::Node poseData;
				readFile(posePath, poseData);
				poseData[dataSection][key] = data;
				writeToFile(posePath, poseData);
				printf(COLOR_PURPLE "    Wrote %s.%s: %d\n" COLOR_RESET, dataSection.c_str(), key.c_str(), data);
			}
			fflush(stdout);
		}
		catch(...) {
			RCLCPP_ERROR(SERIAL_LOGGER, "Failed to write to %s.%s in file %s\n", dataSection.c_str(), key.c_str(), posePath.c_str());
			return false;
		}

		return true;
	}

	float readFloat(const std::string& posePath, const std::string& dataSection, const std::string& key)
	{
		try {
			float val;
			if(posePath == pose_path_sm) {
				val = pose_data_sm[dataSection][key].as<float>();
				printf(COLOR_GRAY "    Fetched %s.%s: %f\n" COLOR_RESET, dataSection.c_str(), key.c_str(), val);
			}
			else {
				YAML::Node poseData;
				readFile(posePath, poseData);
				val = poseData[dataSection][key].as<float>();
				printf(COLOR_PURPLE "    Fetched %s.%s: %f\n" COLOR_RESET, dataSection.c_str(), key.c_str(), val);
			}
			fflush(stdout);
			return val;
		}
		catch(...) {
			RCLCPP_ERROR(SERIAL_LOGGER, "Failed to read %s.%s from file %s\n", dataSection.c_str(), key.c_str(), posePath.c_str());
			return NAN;
		}
	}

	bool writeFloat(const std::string& posePath, const std::string& dataSection, const std::string& key, const float& data)
	{
		try {
			if(posePath == pose_path_sm) {
				pose_data_sm[dataSection][key] = data;
				writeToFile(posePath, pose_data_sm);
				printf(COLOR_GRAY "    Wrote %s.%s: %f\n" COLOR_RESET, dataSection.c_str(), key.c_str(), data);
			}
			else {
				YAML::Node poseData;
				readFile(posePath, poseData);
				poseData[dataSection][key] = data;
				writeToFile(posePath, poseData);
				printf(COLOR_PURPLE "    Wrote %s.%s: %f\n" COLOR_RESET, dataSection.c_str(), key.c_str(), data);
			}
			fflush(stdout);
		}
		catch(...) {
			RCLCPP_ERROR(SERIAL_LOGGER, "Failed to write to %s.%s in file %s\n", dataSection.c_str(), key.c_str(), posePath.c_str());
			return false;
		}

		return true;
	}

	tVectorD readVector(const std::string& posePath, const std::string& dataSection, const std::string& key, tVectorS& poseParm,
						bool silent)
	{
		std::string strValue;
		try {
			if(posePath == pose_path_sm) {
				strValue = pose_data_sm[dataSection][key].as<std::string>();
				printf(COLOR_GRAY "    Fetched %s.%s: %s\n" COLOR_RESET, dataSection.c_str(), key.c_str(), strValue.c_str());
			}
			else {
				YAML::Node poseData;
				readFile(posePath, poseData);
				strValue = poseData[dataSection][key].as<std::string>();
				printf(COLOR_PURPLE "    Fetched %s.%s: %s\n" COLOR_RESET, dataSection.c_str(), key.c_str(), strValue.c_str());
			}
		}
		catch(...) {
			if(!silent)
				RCLCPP_ERROR(SERIAL_LOGGER, "Failed to read %s.%s from file %s\n", dataSection.c_str(), key.c_str(), posePath.c_str());
			return tVectorD();
		}

		std::stringstream poseLine(strValue);
		tVectorD poseVector;
		fflush(stdout);
		while(poseLine.good()) {
			getline(poseLine, strValue, ',');
			boost::algorithm::trim(strValue);
			try {
				if(!strValue.rfind("_", 0))
					poseParm.push_back(strValue);
				else
					poseVector.push_back(std::stod(strValue));
			}
			catch(...) {
				RCLCPP_ERROR(SERIAL_LOGGER, "Invalid pose data %s: %s.%s in %s\n", strValue.c_str(), dataSection.c_str(), key.c_str(),
							 posePath.c_str());
				return tVectorD();
			}
		}

		return poseVector;
	}

	tVectorD readVector(const std::string& posePath, const std::string& dataSection, const std::string& key, bool silent)
	{
		tVectorS parm;
		return readVector(posePath, dataSection, key, parm, silent);
	}

	std::vector<tVectorD> readVectors(const std::string& posePath, const std::string& dataSection, const std::string& key,
									  std::vector<tVectorS>& posesParm)
	{
		std::vector<tVectorD> anglesVector;
		int index = 1;
		while(true) {
			tVectorS parm;
			tVectorD angles = readVector(posePath, dataSection, key + std::to_string(index++), parm, true);
			if(angles.empty())
				break;
			anglesVector.push_back(angles);
			posesParm.push_back(parm);
		}

		return anglesVector;
	}
	std::vector<tVectorD> readListVector(const std::string& posePath, const std::string& dataSection, const std::string& key){
		std::vector<tVectorD> returned_data;
		YAML::Node file_data;
		readFile(posePath, file_data);

		if(!file_data[dataSection][key]) {
			RCLCPP_ERROR(SERIAL_LOGGER, "Failed to read %s.%s from file %s\n", dataSection.c_str(), key.c_str(), posePath.c_str());
			return returned_data;
		}

		// Get the list of vectors for the specified key
		YAML::Node vectorsNode = file_data[dataSection][key];

		// Iterate over the list of vectors and store them in a C++ vector
		for(std::size_t i = 0; i < vectorsNode.size(); ++i) {
			std::vector<double> vec = vectorsNode[i].as<std::vector<double>>();
			returned_data.push_back(vec);
		}

		return returned_data;
	}

	bool fileExists(std::string& fileName)
	{
		if(fileName.size() > 256)
			fileName = fileName.substr(0, 256);
		if(FILE* file = fopen(fileName.c_str(), "r")) {
			fclose(file);
			return true;
		}

		return false;
	}

	bool savePlan(std::string& posePath, const moveit::planning_interface::MoveGroupInterface::Plan& plan)
	{
		if(posePath.size() > 256)
			posePath = posePath.substr(0, 256);
		YAML::Node node;

		// robot state
		// node["start_state"]["joint_state"]["header"]["seq"]			  = plan.start_state_.joint_state.header.seq;
		node["start_state"]["joint_state"]["header"]["stamp"]["sec"]	 = plan.start_state_.joint_state.header.stamp.sec;
		node["start_state"]["joint_state"]["header"]["stamp"]["nanosec"] = plan.start_state_.joint_state.header.stamp.nanosec;
		node["start_state"]["joint_state"]["header"]["frame_id"]		 = plan.start_state_.joint_state.header.frame_id;
		node["start_state"]["joint_state"]["name"]						 = plan.start_state_.joint_state.name;
		node["start_state"]["joint_state"]["position"]					 = plan.start_state_.joint_state.position;
		node["start_state"]["joint_state"]["velocity"]					 = plan.start_state_.joint_state.velocity;
		node["start_state"]["joint_state"]["effort"]					 = plan.start_state_.joint_state.effort;
		node["start_state"]["is_diff"]									 = (int) plan.start_state_.is_diff;

		// node["trajectory"]["joint_trajectory"]["header"]["seq"]			  = plan.trajectory_.joint_trajectory.header.seq;
		node["trajectory"]["joint_trajectory"]["header"]["stamp"]["sec"]	 = plan.trajectory_.joint_trajectory.header.stamp.sec;
		node["trajectory"]["joint_trajectory"]["header"]["stamp"]["nanosec"] = plan.trajectory_.joint_trajectory.header.stamp.nanosec;
		node["trajectory"]["joint_trajectory"]["header"]["frame_id"]		 = plan.trajectory_.joint_trajectory.header.frame_id;

		node["trajectory"]["joint_trajectory"]["joint_names"] = plan.trajectory_.joint_trajectory.joint_names;

		for(int i = 0; i < (int) plan.trajectory_.joint_trajectory.points.size(); i++) {
			// RCLCPP_WARN(SERIAL_LOGGER,"time from start: %f", plan.trajectory_.joint_trajectory.points[i].time_from_start.sec +
			// 									plan.trajectory_.joint_trajectory.points[i].time_from_start.nanosec/1000000000.0);
			YAML::Node point;
			point["positions"]					= plan.trajectory_.joint_trajectory.points[i].positions;
			point["velocities"]					= plan.trajectory_.joint_trajectory.points[i].velocities;
			point["accelerations"]				= plan.trajectory_.joint_trajectory.points[i].accelerations;
			point["effort"]						= plan.trajectory_.joint_trajectory.points[i].effort;
			point["time_from_start"]["sec"]		= plan.trajectory_.joint_trajectory.points[i].time_from_start.sec;
			point["time_from_start"]["nanosec"] = plan.trajectory_.joint_trajectory.points[i].time_from_start.nanosec;

			node["trajectory"]["joint_trajectory"]["points"].push_back(point);
		}

		node["planning_time"] = plan.planning_time_;

		std::ofstream fout(posePath);
		fout << node;

		return true;
	}

	moveit::planning_interface::MoveGroupInterface::Plan loadPlan(std::string& posePath)
	{
		if(posePath.size() > 256)
			posePath = posePath.substr(0, 256);
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		YAML::Node node;
		readFile(posePath, node);

		// plan.start_state_.joint_state.header.seq		= node["start_state"]["joint_state"]["header"]["seq"].as<int>();
		plan.start_state_.joint_state.header.stamp.sec	   = node["start_state"]["joint_state"]["header"]["stamp"]["sec"].as<double>();
		plan.start_state_.joint_state.header.stamp.nanosec = node["start_state"]["joint_state"]["header"]["stamp"]["nanosec"].as<double>();
		plan.start_state_.joint_state.header.frame_id	   = node["start_state"]["joint_state"]["header"]["frame_id"].as<std::string>();
		plan.start_state_.joint_state.name				   = node["start_state"]["joint_state"]["name"].as<std::vector<std::string>>();
		plan.start_state_.joint_state.position			   = node["start_state"]["joint_state"]["position"].as<std::vector<double>>();
		plan.start_state_.joint_state.velocity			   = node["start_state"]["joint_state"]["velocity"].as<std::vector<double>>();
		plan.start_state_.joint_state.effort			   = node["start_state"]["joint_state"]["effort"].as<std::vector<double>>();
		plan.start_state_.is_diff						   = (bool) node["start_state"]["is_diff"].as<int>();

		// plan.trajectory_.joint_trajectory.header.seq	   = node["trajectory"]["joint_trajectory"]["header"]["seq"].as<int>();
		plan.trajectory_.joint_trajectory.header.stamp.sec = node["trajectory"]["joint_trajectory"]["header"]["stamp"]["sec"].as<double>();
		plan.trajectory_.joint_trajectory.header.stamp.nanosec =
			node["trajectory"]["joint_trajectory"]["header"]["stamp"]["nanosec"].as<double>();
		plan.trajectory_.joint_trajectory.header.frame_id = node["trajectory"]["joint_trajectory"]["header"]["frame_id"].as<std::string>();

		plan.trajectory_.joint_trajectory.joint_names =
			node["trajectory"]["joint_trajectory"]["joint_names"].as<std::vector<std::string>>();

		for(YAML::const_iterator it = node["trajectory"]["joint_trajectory"]["points"].begin();
			it != node["trajectory"]["joint_trajectory"]["points"].end(); it++) {
			trajectory_msgs::msg::JointTrajectoryPoint point = it->as<trajectory_msgs::msg::JointTrajectoryPoint>();
			plan.trajectory_.joint_trajectory.points.push_back(point);
		}

		std::sort(plan.trajectory_.joint_trajectory.points.begin(), plan.trajectory_.joint_trajectory.points.end(),
				  [](const auto& lhs, const auto& rhs) {
					  return (lhs.time_from_start.sec + (lhs.time_from_start.nanosec / 1000000000.0)) <
							 (rhs.time_from_start.sec + (rhs.time_from_start.nanosec / 1000000000.0));
				  });

		return plan;
	}

	void deleteAllPlans(const std::string& plansPath)
	{
		std::string cmd = "exec rm -r" + plansPath + "*.yaml";
		int ret			= system(cmd.c_str());
	}

}  // namespace tomo_utils3