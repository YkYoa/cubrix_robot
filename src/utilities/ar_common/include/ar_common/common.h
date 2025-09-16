#pragma once 

#include <string>
#include <vector>
#include <string_view>
#include <unordered_set>
#include <unordered_map>

namespace ar_common
{
    // For robot scene directory
    std::string_view getRobotSceneDirectory();

    // For Planner Utilities
    const std::unordered_set<std::string_view>& getOMPLPlanners();
    const std::unordered_set<std::string_view>& getPILZPlanners();
    const std::unordered_map<std::string_view, 
                         std::unordered_set<std::string_view>>& getPlannerPipelines();
    std::string_view getDefaultPlannerPipeline();
    std::string_view getDefaultPlannerID();
    double getDefaultPlanningTime();

    // For joint/link definitions

    const std::vector<std::string>& getArmJoints();
    std::string_view getArmEndEffector();
    std::string_view getJointGroupARM();
    const std::vector<std::string>& getJointGroups();
    
    // Config path

    void getConfigPath();
    void readYamlFile(std::string yaml_path);
    void readYamlFile(std::string yaml_path, int slaveId);

}// namespace ar_common
