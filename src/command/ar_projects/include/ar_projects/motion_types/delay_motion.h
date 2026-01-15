#ifndef AR_PROJECTS__MOTION_TYPES__DELAY_MOTION_H_
#define AR_PROJECTS__MOTION_TYPES__DELAY_MOTION_H_

#include "ar_projects/motion_types/base_motion.h"

namespace ar_projects
{

/**
 * @brief Delay/wait motion
 * 
 * YAML format:
 *   - type: "delay"
 *     duration_ms: 500
 *     name: "Wait"
 */
class DelayMotion : public BaseMotion
{
public:
  /**
   * @brief Constructor
   */
  DelayMotion();
  
  /**
   * @brief Convert to BehaviorTree XML
   * @param indent Indentation level
   * @return XML string
   */
  std::string toXml(int indent = 0) const override;
  
  /**
   * @brief Parse from YAML node
   * @param node YAML node containing delay configuration
   * @return true if parsing succeeded
   */
  bool fromYaml(const YAML::Node& node) override;

  /**
   * @brief Set delay duration in milliseconds
   * @param duration Duration in milliseconds
   */
  void setDurationMs(int duration) { duration_ms_ = duration; }
  
  /**
   * @brief Get delay duration in milliseconds
   * @return Duration in milliseconds
   */
  int getDurationMs() const { return duration_ms_; }

private:
  int duration_ms_ = 500;
};

}  // namespace ar_projects

#endif  // AR_PROJECTS__MOTION_TYPES__DELAY_MOTION_H_
