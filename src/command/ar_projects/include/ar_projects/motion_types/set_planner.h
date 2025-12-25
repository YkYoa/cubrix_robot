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
  SetPlannerMotion();
  
  std::string toXml(int indent = 0) const override;
  bool fromYaml(const YAML::Node& node) override;

  void setPipeline(const std::string& pipeline) { pipeline_ = pipeline; }
  void setPlannerId(const std::string& planner_id) { planner_id_ = planner_id; }
  
  const std::string& getPipeline() const { return pipeline_; }
  const std::string& getPlannerId() const { return planner_id_; }

private:
  std::string pipeline_ = "pilz";
  std::string planner_id_ = "PTP";
};

}  // namespace ar_projects

#endif  // AR_PROJECTS__MOTION_TYPES__SET_PLANNER_H_
