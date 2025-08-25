#include "ar_common/ar_scene_definition.h"
#include "ar_common/joint_link_definition.h"
#include "ar_common/planner_definition.h"



namespace ar
{
    std::string_view getRobotSceneDirectory()
    {
        return SCENE_ROBOT_DESC;
    }

    const std::unordered_set<std::string_view>& getOMPLPlanners()
    {
        return planner::OMPL_LIST;
    }

    const std::unordered_set<std::string_view>& getPILZPlanners()
    {
        return planner::PILZ_LIST;
    }

    const std::unordered_map<std::string_view, 
                         std::unordered_set<std::string_view>>& getPlannerPipelines()
    {
        return planner::PIPELINE_LIST;
    }

    std::string_view getDefaultPlannerPipeline()
    {
        return planner::PIPELINE_DEFAULT;
    }

    std::string_view getDefaultPlannerID()
    {
        return planner::PLAN_ID_DEFAULT;
    }

    double getDefaultPlanningTime()
    {
        return planner::PLAN_TIME_DEFAULT;
    }

    const std::vector<std::string>& getArmJoints()
    {
        return joint::ARM;
    }

    const std::vector<std::string>& getJointsGroups()
    {
        return JointGroups;
    }

    std::string_view getArmEndEffector()
    {
        return link::ENDEFF_ARM;
    }

    std::string_view getJointGroupARM()
    {
        return jointgroup::ARM;
    }


} // namespace ar