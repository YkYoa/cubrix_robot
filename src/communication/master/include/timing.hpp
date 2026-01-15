#pragma once
#include <vector>
#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <cstdint>
#include <ctime>
#include <ratio>
#include <fstream>
#include <string>
#include <iostream>

#define NUMBER_OF_SAMPLES 1E6
/**
 *  \class   Timing
 *  \brief   Contains Timing measurement related functions.
 */
/**
 * @brief Timing measurement utility class
 * 
 * Provides functionality to measure and log timing information
 * for performance analysis and loop timing verification.
 */
class Timing{
    public:
      std::chrono::high_resolution_clock::time_point timer_start_; ///< Start time point
      std::chrono::high_resolution_clock::time_point last_start_time_; ///< Last start time
      std::chrono::duration<long,std::micro> time_span_; ///< Time span measurement
      std::vector<long> timing_info_ = std::vector<long>(NUMBER_OF_SAMPLES); ///< Timing data storage
      uint32_t counter_ = 0; ///< Sample counter
      
  /**
   * @brief Get current time and assign to timer_start_ member
   */
  void GetTime();
  
  /**
   * @brief Measure time difference from last GetTime() call
   * 
   * Calculates the time difference and stores it in time_span member.
   */
  void MeasureTimeDifference();
  
  /**
   * @brief Output timing information to file
   * 
   * Writes collected timing data to loop_timing_info.txt file.
   */
  void OutInfoToFile();
};