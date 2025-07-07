#include "ar_utils.h"

using namespace std::chrono;
// Udupa; Feb'22; Threadsafe timestamp with ms precision

namespace ar_utils{
	std::string getCurrentTime(bool includeDate, bool timeSinceEpoch)
	{
		char timeBuf[50];
		if(!includeDate && timeSinceEpoch) {
			std::snprintf(timeBuf, sizeof(timeBuf), " [%.9f]",
						  duration_cast<std::chrono::nanoseconds>(system_clock::now().time_since_epoch()).count() / 1000000000.0);
		}
		else {
			struct timeval tv;
			gettimeofday(&tv, NULL);
			struct tm* tm = localtime(&tv.tv_sec);
			if(includeDate)
				std::snprintf(timeBuf, sizeof(timeBuf), "%04d-%02d-%02d_%02d:%02d:%02d", 1900 + tm->tm_year, tm->tm_mon + 1, tm->tm_mday,
							  tm->tm_hour, tm->tm_min, tm->tm_sec);
			else
				std::snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d.%03d", tm->tm_hour, tm->tm_min, tm->tm_sec,
							  (int) (tv.tv_usec / 1000));
		}

		return std::string(timeBuf);
	}

	std::string getHomeDirectory()
	{
		std::string home_dir;
		if(std::getenv("SUDO_USER") != nullptr) {
			home_dir = "/home/" + std::string(std::getenv("SUDO_USER"));
		}
		else {
			home_dir = getenv("HOME");
		}
		return home_dir;
	}

	int stringToId(std::string str)
    {
        int index;
        size_t pos = str.find("_");
        if(pos != std::string::npos && pos + 1 < str.length()){
            std::string id_str = str.substr(pos + 1);
            index = std::stoi(id_str);        
        }
        return index;
    }
}