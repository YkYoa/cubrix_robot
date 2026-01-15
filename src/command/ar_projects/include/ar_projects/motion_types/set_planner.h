#ifndef AR_PROJECTS__MOTION_TYPES__SET_PLANNER_H_
#define AR_PROJECTS__MOTION_TYPES__SET_PLANNER_H_

#include "ar_projects/motion_types/base_motion.h"

namespace ar_projects
{

/**
 * @brief Set planner configuration
 * 
 * YAML format:
 *   - type: "set_planner"
 *     pipeline: "pilz"
 *     planner_id: "PTP"
 */
class SetPlannerMotion : public BaseMotion
{
public:
  /**
   * @brief Constructor
   */
  SetPlannerMotion();
  
  /**
   * @brief Convert to BehaviorTree XML
   * @param indent Indentation level
   * @return XML string
   */
  std::string toXml(int indent = 0) const override;
  
  /**
   * @brief Parse from YAML node
   * @param node YAML node containing set_planner configuration
   * @return true if parsing succeeded
   */
  bool fromYaml(const YAML::Node& node) override;

  /**
   * @brief Set planning pipeline ID
   * @param pipeline Pipeline ID (e.g., "ompl", "pilz")
   */
  void setPipeline(const std::string& pipeline) { pipeline_ = pipeline; }
  
  /**
   * @brief Set planner ID
   * @param planner_id Planner ID (e.g., "RRTConnect", "PTP")
   */
  void setPlannerId(const std::string& planner_id) { planner_id_ = planner_id; }
  
  /**
   * @brief Get planning pipeline ID
   * @return Pipeline ID string
   */
  const std::string& getPipeline() const { return pipeline_; }
  
  /**
   * @brief Get planner ID
   * @return Planner ID string
   */
  const std::string& getPlannerId() const { return planner_id_; }

private:
  std::string pipeline_ = "pilz";
  std::string planner_id_ = "PTP";
};

}  // namespace ar_projects

#endif  // AR_PROJECTS__MOTION_TYPES__SET_PLANNER_H_
