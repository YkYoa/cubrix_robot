/// Serialization functions collection; supports reading of different data structures from yaml files
/// Udupa; Apr'22
/// Added data caching option for optimization; Udupa; Jul'22
/// Added write data functions; Udupa; Jul'22

#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <ar_utils/ar_utils.h>

namespace ar_utils
{
	extern bool cache_data;
	void clearFileCache(const std::string& filePath = "");

	std::string readString(const std::string& posePath, const std::string& dataSection, const std::string& key);
	bool writeString(const std::string& posePath, const std::string& dataSection, const std::string& key, const std::string& data);

	int readInt(const std::string& posePath, const std::string& dataSection, const std::string& key);
	bool writeInt(const std::string& posePath, const std::string& dataSection, const std::string& key, const int& data);

	float readFloat(const std::string& posePath, const std::string& dataSection, const std::string& key);
	bool writeFloat(const std::string& posePath, const std::string& dataSection, const std::string& key, const float& data);

	tVectorD readVector(const std::string& posePath, const std::string& dataSection, const std::string& key, tVectorS& poseParm,
						bool silent = false);
	tVectorD readVector(const std::string& posePath, const std::string& dataSection, const std::string& key, bool silent = false);
	std::vector<tVectorD> readVectors(const std::string& posePath, const std::string& dataSection, const std::string& key,
									  std::vector<tVectorS>& posesParm);

	// List vector following form
	// dataSection:
	// 	key:
	// 		- [0.0, 0.0, 0.0, ...]
	// 		- [0.0, 0.0, 0.0, ...]
	// 		- [0.0, 0.0, 0.0, ...]
	std::vector<tVectorD> readListVector(const std::string& posePath, const std::string& dataSection, const std::string& key);

	bool fileExists(std::string& fileName);
	bool savePlan(std::string& posePath, const moveit::planning_interface::MoveGroupInterface::Plan& plan);
	moveit::planning_interface::MoveGroupInterface::Plan loadPlan(std::string& posePath);
	void deleteAllPlans(const std::string& plansPath);
	void readFileSm(std::string pose_path);
}  // namespace ar_utils

struct TrajectoryPoint
{
	std::vector<double> positions;
	std::vector<double> velocities;
	std::vector<double> accelerations;
	std::vector<double> effort;
	double sec;
	double nanosec;
};
namespace YAML
{
	template <> struct convert<trajectory_msgs::msg::JointTrajectoryPoint>
	{
		static Node encode(const trajectory_msgs::msg::JointTrajectoryPoint& rhs)
		{
			Node point;

			point["positions"]					= rhs.positions;
			point["velocities"]					= rhs.velocities;
			point["accelerations"]				= rhs.accelerations;
			point["effort"]						= rhs.effort;
			point["time_from_start"]["sec"]		= rhs.time_from_start.sec;
			point["time_from_start"]["nanosec"] = rhs.time_from_start.nanosec;

			return point;
		}

		static bool decode(const Node& point, trajectory_msgs::msg::JointTrajectoryPoint& rhs)
		{
			// if(!point.IsSequence()) {
			// 	return false;
			// }

			rhs.positions				= point["positions"].as<std::vector<double>>();
			rhs.velocities				= point["velocities"].as<std::vector<double>>();
			rhs.accelerations			= point["accelerations"].as<std::vector<double>>();
			rhs.effort					= point["effort"].as<std::vector<double>>();
			rhs.time_from_start.sec		= point["time_from_start"]["sec"].as<double>();
			rhs.time_from_start.nanosec = point["time_from_start"]["nanosec"].as<double>();
			return true;
		}
	};
}  // namespace YAML
