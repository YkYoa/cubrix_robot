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
  DelayMotion();
  
  std::string toXml(int indent = 0) const override;
  bool fromYaml(const YAML::Node& node) override;

  void setDurationMs(int duration) { duration_ms_ = duration; }
  int getDurationMs() const { return duration_ms_; }

private:
  int duration_ms_ = 500;
};

}  // namespace ar_projects

#endif  // AR_PROJECTS__MOTION_TYPES__DELAY_MOTION_H_
