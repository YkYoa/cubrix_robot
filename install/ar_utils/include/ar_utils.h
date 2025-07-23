#pragma once

#include <chrono>
#include <stdint.h>
#include <string>
#include <sys/time.h>
#include <vector>

typedef std::vector<double> tVectorD;
typedef std::vector<float> tVectorF;
typedef std::vector<tVectorD> tVectorDs;
typedef std::vector<std::string> tVectorS;
typedef std::vector<int> tVectorI;
typedef std::vector<uint8_t> tVectorU;
typedef std::vector<bool> tVectorB;

#define COLOR_GRAY		 "\033[90m"
#define COLOR_RED		 "\033[91m"
#define COLOR_GREEN		 "\033[92m"
#define COLOR_YELLOW	 "\033[93m"
#define COLOR_DARKYELLOW "\033[0;33m"
#define COLOR_BLUE		 "\033[94m"
#define COLOR_PURPLE	 "\033[95m"
#define COLOR_CYAN		 "\033[96m"
#define COLOR_RESET		 "\033[0m"


namespace ar_utils{
	std::string getCurrentTime(bool includeDate = false, bool timeSinceEpoch = true);
	std::string getHomeDirectory();
	int stringToId(std::string str);
}