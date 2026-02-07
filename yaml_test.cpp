#include <iostream>
#include <yaml-cpp/yaml.h>

int main() {
    std::string yaml_data = R"(
joints:
  Arm_joint3:
    id: 3
  Arm_joint1:
    id: 1
)";
    
    YAML::Node config = YAML::Load(yaml_data);
    YAML::Node joints = config["joints"];
    
    std::cout << "Iteration Order:\n";
    for(YAML::const_iterator it = joints.begin(); it != joints.end(); ++it) {
        std::cout << it->first.as<std::string>() << "\n";
    }
    return 0;
}
