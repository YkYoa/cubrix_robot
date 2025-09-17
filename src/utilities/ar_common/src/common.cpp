#include "ar_common/ar_definition.h"
#include "ar_common/joint_link_definition.h"
#include "ar_common/planner_definition.h"
#include "ar_common/common.h"
#include <iostream>



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

namespace ar_common
{
    std::string getConfigPath()
    {
        return ament_index_cpp::get_package_share_directory("ar_control") + ROBOT_YAML_DEFINITION;
    }

    YAML::Node readYamlFile(const std::string& yaml_path)
    {
        YAML::Node config;
        try
        {
            config = YAML::LoadFile(yaml_path);
        }
        catch (const YAML::BadFile &e)
        {
            std::cout << "Error: Failed to load the YAML file: " << e.what() << std::endl;
            return YAML::Node();
        }
        catch (const YAML::ParserException &e)
        {
            std::cout << "Error: YAML parsing error: " << e.what() << std::endl;
            return YAML::Node();
        }
        catch (const std::exception &e)
        {
            std::cout << "Error: An unexpected error occurred: " << e.what() << std::endl;
            return YAML::Node();
        }
        return config;
    }

    YAML::Node readYamlFile()
    {
        return readYamlFile(getConfigPath());
    }

} // namespace ar_common