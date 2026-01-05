#ifndef AR_MOTION_PLANNER__PLANNER_TYPES_H_
#define AR_MOTION_PLANNER__PLANNER_TYPES_H_

#include <cstdint>
#include <string>

namespace ar_motion_planner
{

/**
 * @brief Type of planner request
 */
enum class PlannerRequestType : int8_t {
  UNKNOWN = -1,
  MOTION,
  OBJECT,
  SCENE
};

/**
 * @brief Scene object operation type
 */
enum class Operation : uint8_t {
  ADD,
  ADD_ATTACH,
  REMOVE,
  MOVE,
  ATTACH,
  DETACH,
  DETACH_ATTACH
};

/**
 * @brief Object request parameters for scene operations
 */
struct ObjectRequest {
  std::string object_label = "";
  std::string mesh_name = "";
  std::string link_name = "";
};

} // namespace ar_motion_planner

#endif // AR_MOTION_PLANNER__PLANNER_TYPES_H_
