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
  /**
   * @brief Constructor
   */
  SetVelocityMotion();
  
  /**
   * @brief Convert to BehaviorTree XML
   * @param indent Indentation level
   * @return XML string
   */
  std::string toXml(int indent = 0) const override;
  
  /**
   * @brief Parse from YAML node
   * @param node YAML node containing set_velocity configuration
   * @return true if parsing succeeded
   */
  bool fromYaml(const YAML::Node& node) override;

  /**
   * @brief Set velocity scaling factor
   * @param factor Scaling factor (0.0 to 1.0)
   */
  void setFactor(double factor) { factor_ = factor; }
  
  /**
   * @brief Get velocity scaling factor
   * @return Scaling factor
   */
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
  /**
   * @brief Constructor
   */
  SetAccelerationMotion();
  
  /**
   * @brief Convert to BehaviorTree XML
   * @param indent Indentation level
   * @return XML string
   */
  std::string toXml(int indent = 0) const override;
  
  /**
   * @brief Parse from YAML node
   * @param node YAML node containing set_acceleration configuration
   * @return true if parsing succeeded
   */
  bool fromYaml(const YAML::Node& node) override;

  /**
   * @brief Set acceleration scaling factor
   * @param factor Scaling factor (0.0 to 1.0)
   */
  void setFactor(double factor) { factor_ = factor; }
  
  /**
   * @brief Get acceleration scaling factor
   * @return Scaling factor
   */
  double getFactor() const { return factor_; }

private:
  double factor_ = 0.5;
};

}  // namespace ar_projects

#endif  // AR_PROJECTS__MOTION_TYPES__SET_VELOCITY_H_
