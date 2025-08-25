#pragma once 


#include <stdint.h>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
namespace ar
{
	namespace planner{
		/**
		 * @file  planner_definition.h
		 * @brief all the defined constants for planner
		 *
		 *  if there are new constant in planner intrface plackage, it should be added to this file,
		 *  and a easy to understand name should be given to prevent confusion.
		 * 	naming convention we can follow <planner class name>_<value_name>_<value attribute>
		 * 	plase use inline constexpr as much as possible : https://www.geeksforgeeks.org/understanding-constexper-specifier-in-cpp/
		 * 	why use string_view instead of string: https://stackoverflow.com/questions/20803826/what-is-string-view
		 */
		// Planner pipeline
		inline constexpr std::string_view PIPELINE_OMPL = "ompl";
		inline constexpr std::string_view PIPELINE_PILZ = "pilz";
		// Planner_id
		// OMPL
		inline constexpr std::string_view OMPL_RRTC	= "RRTConnectkConfigDefault";  // low cost, low solution accuracy
		inline constexpr std::string_view OMPL_BITRRT	= "BiTRRTkConfigDefault";	   // cost aware, medium solution accuracy
		inline constexpr std::string_view OMPL_RRTSTAR = "RRTstarkConfigDefault";	   // high cost, high solution accuracy
		// PILZ
		inline constexpr std::string_view PILZ_PTP	   = "PTP";
		inline constexpr std::string_view PILZ_LINEAR = "LIN";
		inline constexpr std::string_view PILZ_CIRC   = "CIRC";
		// Planner base default configurations
		inline constexpr double MAX_VEL_SCALE_DEFAULT		= 0.1;
		inline constexpr double MAX_ACC_SCALE_DEFAULT		= 0.1;
		inline constexpr double GOAL_TOLERANCE_DEFAULT		= 0.001;
		inline constexpr uint16_t PLAN_ATTEMPT_DEFAULT		= 3;
		inline constexpr double PLAN_TIME_DEFAULT			= 5.0;
		inline constexpr std::string_view PIPELINE_DEFAULT = PIPELINE_PILZ;
		inline constexpr std::string_view PLAN_ID_DEFAULT	= PILZ_PTP;
		// Lists
		inline const std::unordered_set<std::string_view> OMPL_LIST = {OMPL_RRTC, OMPL_BITRRT, OMPL_RRTSTAR};
		inline const std::unordered_set<std::string_view> PILZ_LIST = {PILZ_PTP, PILZ_LINEAR, PILZ_CIRC};
		inline const std::unordered_map<std::string_view, std::unordered_set<std::string_view>> PIPELINE_LIST = {
			{PIPELINE_OMPL, OMPL_LIST}, {PIPELINE_PILZ, PILZ_LIST}};
	}// namespace planner

	namespace planner_manager
	{
		inline constexpr uint16_t SUBSCRIBER_SLEEP_TIME = 	100;  // milliseconds
	}// namespace planner_manager

}  // namespace ar