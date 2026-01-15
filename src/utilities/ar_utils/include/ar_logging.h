#ifndef AR_LOGGING_H
#define AR_LOGGING_H

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>

#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>


typedef std::chrono::high_resolution_clock::time_point TimeVar;


#define LOG_FLAG

#ifdef LOG_FLAG

#define LOG_CALL(func, ar_log, log_name)                                                            \
	{                                                                                                 \
		timespec start_time = ar_utils::ArLogger::now();                                          \
		func;                                                                                         \
		timespec end_time = ar_utils::ArLogger::now();                                            \
		ar_log.log_file.open(ar_log.log_path, std::ios::app);                                     \
		ar_log.log_file << log_name << "," << start_time.tv_sec << "," << start_time.tv_nsec << "," \
						  << ar_utils::ArLogger::calculateDelta(start_time, end_time) << "\n";    \
		ar_log.log_file.close();                                                                    \
	}                                                                                                 \
	// std::cout << "time eplasped: " << end - start << std::endl

#define LOG_CALL_RE(func, ar_log, log_name, func_return)                                            \
	{                                                                                                 \
		timespec start_time = ar_utils::ArLogger::now();                                          \
		func_return			= func;                                                                   \
		timespec end_time	= ar_utils::ArLogger::now();                                          \
		ar_log.log_file.open(ar_log.log_path, std::ios::app);                                     \
		ar_log.log_file << log_name << "," << start_time.tv_sec << "," << start_time.tv_nsec << "," \
						  << ar_utils::ArLogger::calculateDelta(start_time, end_time) << "\n";    \
		ar_log.log_file.close();                                                                    \
	}                                                                                                 \
	// std::cout << "time eplasped: " << end - start << std::endl

#else
#define LOG_CALL(func, log_file, log_name) func
#endif

namespace ar_utils
{

	/**
	 * @brief Logger for timing and performance measurements
	 * 
	 * Provides functionality to log execution times, joint values,
	 * and other performance metrics to CSV files.
	 */
	class ArLogger
	{
	public:
		/**
		 * @brief Default constructor
		 */
		ArLogger();
		
		/**
		 * @brief Constructor with log file name
		 * @param fileName Path to log file
		 */
		ArLogger(std::string fileName);
		
		/**
		 * @brief Destructor
		 */
		~ArLogger();
		
		/**
		 * @brief Open a log file
		 * @param fileName Path to log file
		 */
		void open(std::string fileName);

		/**
		 * @brief Get current timestamp
		 * @return Current time as timespec
		 */
		static timespec now();
		
		/**
		 * @brief Calculate time difference in seconds
		 * @param startTime Start time
		 * @param end_time End time
		 * @return Time difference in seconds
		 */
		static double calculateDelta(const timespec& startTime, timespec end_time);
		// template <typename F, typename... Args> double logTime(std::string log_name, F func, Args&&... args);
		// template <typename F, typename... Args> auto logTime(std::string log_name, F func, Args&&... args)
		// {
		// 	double t1 = now();
		// 	// std::cout << log_name << std::endl;
		// 	// std::cout << std::put_time(std::localtime(&t_c), "%F %T.\n") << std::endl;
		// 	auto result = func(std::forward<Args>(args)...);
		// 	double time_elapsed = now() - t1;
		// 	if(log_flag) {
		// 		std::lock_guard<std::mutex> guard(mu);
		// 		log_file << log_name << "," << t1 << "," << time_elapsed << "\n";
		// 	}
		// 	return result;
		// }

		/**
		 * @brief Start timing
		 */
		void start();
		
		/**
		 * @brief Get start time
		 * @return Start time
		 */
		timespec getStartTime();
		
		/**
		 * @brief Get duration since start
		 * @return Duration in seconds
		 */
		double getDuration();
		
		/**
		 * @brief Convert timespec to seconds
		 * @param startTime Time to convert
		 * @return Time in seconds
		 */
		double toSecond(const timespec& startTime);
		
		/**
		 * @brief Get duration from start time to now
		 * @param startTime Start time
		 * @return Duration in seconds
		 */
		static double getDuration(const timespec& startTime);
		
		/**
		 * @brief Log duration in seconds
		 * @param log_name Log entry name
		 * @return Duration in seconds
		 */
		double logDurationInSecond(const std::string& log_name);
		
		/**
		 * @brief Log duration in milliseconds
		 * @param log_name Log entry name
		 * @return Duration in milliseconds
		 */
		double logDurationInMilisecond(const std::string& log_name);
		
		/**
		 * @brief Log duration with custom start time
		 * @param log_name Log entry name
		 * @param startTime Start time (default: {0, 0} uses internal start time)
		 * @return Duration in seconds
		 */
		double logDuration(const std::string& log_name, const timespec& startTime = {0, 0});
		
		/**
		 * @brief Log duration with period check
		 * @param log_name Log entry name
		 * @param period Expected period in seconds
		 * @return Duration in seconds
		 */
		double logDuration(const std::string& log_name, const double period);
		
		/**
		 * @brief Log joint values
		 * @param group_name Joint group name
		 * @param pose_name Pose name
		 * @param joints Joint values vector
		 * @return true if logged successfully
		 */
		bool logJointValues(const std::string& group_name, const std::string& pose_name, const std::vector<double> joints);

		std::ofstream log_file;


		std::mutex mu;
		std::string log_path;

	private:
		timespec start_time;
		timespec end_time;
	};
}  // namespace ar_utils

#endif	// AR_LOGGING_H