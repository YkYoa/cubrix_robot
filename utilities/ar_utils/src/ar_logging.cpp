#include "ar_logging.h"
#include "ar_utils.h"

#define NSECS_PER_SECOND 1000000000.0
auto LOGGER = rclcpp::get_logger("ar_logging");
namespace ar_utils
{
	ArLogger::ArLogger()
	{
		log_path = "";
	}

	ArLogger::ArLogger(std::string fileName)
	{
		open(fileName);
	}

	void ArLogger::open(std::string fileName)
	{
		std::string homeDir = ar_utils::getHomeDirectory();
		log_path			= homeDir + "/ar_stats/log/timer_" + fileName + ".csv";

		log_file.open(log_path, std::ios::app);
		if(log_file.is_open()) {
			log_file << "name,start_time_sec,start_time_nsec,time_elapsed\n";
			log_file.close();
			RCLCPP_INFO(LOGGER, "Successfully open file %s for ar_logging", log_path.c_str());
		}
		else {
			RCLCPP_ERROR(LOGGER, "Can not open file %s for ar_logging\n", log_path.c_str());
			log_path = "";
		}
	}

	ArLogger::~ArLogger()
	{
		if(log_file.is_open())
			log_file.close();
	}

	timespec ArLogger::now()
	{
		struct timespec n;
		clock_gettime(CLOCK_REALTIME, &n);
		// cur_time.push_back(n.tv_sec);
		// cur_time.push_back(n.tv_nsec);
		return n;
	}

	double ArLogger::calculateDelta(const timespec& startTime, timespec endTime)
	{
		return (static_cast<double>(endTime.tv_nsec) / NSECS_PER_SECOND + endTime.tv_sec) -
			   (static_cast<double>(startTime.tv_nsec) / NSECS_PER_SECOND + startTime.tv_sec);
	}

	double ArLogger::toSecond(const timespec& startTime)
	{
		return (static_cast<double>(startTime.tv_nsec) / NSECS_PER_SECOND + startTime.tv_sec);
	}

	void ArLogger::start()
	{
		start_time = now();
	}
	timespec ArLogger::getStartTime()
	{
		return start_time;
	}

	double ArLogger::getDuration(const timespec& startTime)
	{
		return calculateDelta(startTime, now());
	}

	double ArLogger::getDuration()
	{
		return getDuration(start_time);
	}

	double ArLogger::logDurationInSecond(const std::string& log_name)
	{
		return logDuration(log_name, toSecond(start_time));
	}

	double ArLogger::logDurationInMilisecond(const std::string& log_name)
	{
		return logDuration(log_name, toSecond(start_time) * 1000);
		// start_time = {0, 0};
	}

	double ArLogger::logDuration(const std::string& log_name, const timespec& startTime)
	{

		if(startTime.tv_sec != 0)
			start_time = startTime;

		double time_elapsed = calculateDelta(start_time, now());
		if(!log_path.empty()) {
			// std::lock_guard<std::mutex> guard(mu);
			log_file.open(log_path, std::ios::app);
			log_file << log_name << "," << start_time.tv_sec << "," << start_time.tv_nsec << "," << time_elapsed << "\n";
			log_file.close();
		}
		return time_elapsed;
	}

	double ArLogger::logDuration(const std::string& log_name, const double period)
	{
		if(!log_path.empty()) {
			// std::lock_guard<std::mutex> guard(mu);
			log_file.open(log_path, std::ios::app);
			log_file << log_name << "," << 0 << "," << 0 << "," << period << "\n";
			log_file.close();
		}
		return period;
	}

	bool ArLogger::logJointValues(const std::string& group_name, const std::string& pose_name, const std::vector<double> joints)
	{
		if(!log_path.empty()) {
			// std::lock_guard<std::mutex> guard(mu);
			log_file.open(log_path, std::ios::app);
			// log_file << log_name << "," << 0 << "," << 0 << "," << period << "\n";
			log_file << group_name << "," << pose_name;
			for(auto joint : joints) {
				log_file << "," << joint;
			}
			log_file << "\n";

			log_file.close();
		}
		return true;
	}
}  // namespace ar_utils