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

	class ArLogger
	{
	public:
		ArLogger();
		ArLogger(std::string fileName);
		~ArLogger();
		void open(std::string fileName);

		static timespec now();
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

		void start();
		timespec getStartTime();
		double getDuration();
		double toSecond(const timespec& startTime);
		static double getDuration(const timespec& startTime);
		double logDurationInSecond(const std::string& log_name);
		double logDurationInMilisecond(const std::string& log_name);
		double logDuration(const std::string& log_name, const timespec& startTime = {0, 0});
		double logDuration(const std::string& log_name, const double period);
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