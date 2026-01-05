#ifndef AR_PROJECTS__YAML_GENERATOR_H_
#define AR_PROJECTS__YAML_GENERATOR_H_

#include <string>
#include "ar_projects/yaml_parser.h"

namespace ar_projects
{

/**
 * @brief Generate YAML config from project configuration
 */
class YamlGenerator
{
public:
  YamlGenerator() = default;
  ~YamlGenerator() = default;

  /**
   * @brief Generate YAML from project config
   * @param config Project configuration
   * @return YAML string
   */
  std::string generate(const ProjectConfig& config);

  /**
   * @brief Write generated YAML to file
   * @param config Project configuration
   * @param output_filepath Output file path
   * @return true if successful
   */
  bool writeToFile(const ProjectConfig& config, const std::string& output_filepath);

  /**
   * @brief Get last generation error
   */
  const std::string& getError() const { return error_; }

private:
  std::string error_;
};

}  // namespace ar_projects

#endif  // AR_PROJECTS__YAML_GENERATOR_H_
