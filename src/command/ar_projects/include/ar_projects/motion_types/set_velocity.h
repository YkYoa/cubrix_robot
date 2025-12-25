#ifndef AR_PROJECTS__MOTION_TYPES__SET_VELOCITY_H_
#define AR_PROJECTS__MOTION_TYPES__SET_VELOCITY_H_

#include "ar_projects/motion_types/base_motion.h"

namespace ar_projects
{

/**
 * @brief Set velocity scaling
 * 
 * YAML format:
 *   - type: "set_velocity"
 *     factor: 0.5
 */
class SetVelocityMotion : public BaseMotion
{
public:
  SetVelocityMotion();
  
  std::string toXml(int indent = 0) const override;
  bool fromYaml(const YAML::Node& node) override;

  void setFactor(double factor) { factor_ = factor; }
  double getFactor() const { return factor_; }

private:
  double factor_ = 0.5;
};

/**
 * @brief Set acceleration scaling
 * 
 * YAML format:
 *   - type: "set_acceleration"
 *     factor: 0.5
 */
class SetAccelerationMotion : public BaseMotion
{
public:
  SetAccelerationMotion();
  
  std::string toXml(int indent = 0) const override;
  bool fromYaml(const YAML::Node& node) override;

  void setFactor(double factor) { factor_ = factor; }
  double getFactor() const { return factor_; }

private:
  double factor_ = 0.5;
};

}  // namespace ar_projects

#endif  // AR_PROJECTS__MOTION_TYPES__SET_VELOCITY_H_
