#ifndef AR_PROJECTS__XML_GENERATOR_H_
#define AR_PROJECTS__XML_GENERATOR_H_

#include <string>
#include "ar_projects/yaml_parser.h"

namespace ar_projects
{

/**
 * @brief Generate BehaviorTree XML from project configuration
 */
class XmlGenerator
{
public:
  XmlGenerator();
  ~XmlGenerator() = default;

  /**
   * @brief Generate XML from project config
   * @param config Project configuration
   * @return XML string
   */
  std::string generate(const ProjectConfig& config);

  /**
   * @brief Generate XML from YAML file
   * @param yaml_filepath Path to YAML file
   * @return XML string
   */
  std::string generateFromFile(const std::string& yaml_filepath);

  /**
   * @brief Write generated XML to file
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
  /**
   * @brief Generate XML header
   */
  std::string generateHeader() const;

  /**
   * @brief Generate XML footer
   */
  std::string generateFooter() const;

  /**
   * @brief Generate default settings nodes
   */
  std::string generateDefaults(const ProjectConfig& config, int indent) const;

  /**
   * @brief Generate motion sequence
   */
  std::string generateSequence(const ProjectConfig& config, int indent) const;

  std::string error_;
  YamlParser parser_;
};

}  // namespace ar_projects

#endif  // AR_PROJECTS__XML_GENERATOR_H_
