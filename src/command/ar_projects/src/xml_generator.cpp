#include "ar_projects/xml_generator.h"
#include <fstream>
#include <sstream>

namespace ar_projects
{

XmlGenerator::XmlGenerator()
{
}

std::string XmlGenerator::generate(const ProjectConfig& config)
{
  std::ostringstream xml;
  
  xml << generateHeader();
  xml << "  <BehaviorTree ID=\"" << config.name << "\">\n";
  xml << "    <Sequence name=\"" << config.name << "Sequence\">\n";
  
  // Generate default settings
  xml << generateDefaults(config, 3);
  
  // Generate motion sequence
  xml << generateSequence(config, 3);
  
  xml << "    </Sequence>\n";
  xml << "  </BehaviorTree>\n";
  xml << generateFooter();
  
  return xml.str();
}

std::string XmlGenerator::generateFromFile(const std::string& yaml_filepath)
{
  error_.clear();
  
  auto config = parser_.parse(yaml_filepath);
  if (!parser_.getError().empty()) {
    error_ = parser_.getError();
    return "";
  }
  
  return generate(config);
}

bool XmlGenerator::writeToFile(const ProjectConfig& config, const std::string& output_filepath)
{
  error_.clear();
  
  std::string xml = generate(config);
  if (xml.empty()) {
    error_ = "Generated XML is empty";
    return false;
  }
  
  std::ofstream file(output_filepath);
  if (!file.is_open()) {
    error_ = "Could not open file for writing: " + output_filepath;
    return false;
  }
  
  file << xml;
  file.close();
  
  return true;
}

std::string XmlGenerator::generateHeader() const
{
  return "<root BTCPP_format=\"4\">\n";
}

std::string XmlGenerator::generateFooter() const
{
  return "</root>\n";
}

std::string XmlGenerator::generateDefaults(const ProjectConfig& config, int ind) const
{
  std::ostringstream xml;
  std::string indent_str(ind * 2, ' ');
  
  // Add comment
  xml << indent_str << "<!-- Default configuration -->\n";
  
  // Set planner
  xml << indent_str << "<SetPlanner pipeline=\"" << config.default_pipeline 
      << "\" planner=\"" << config.default_planner << "\"/>\n";
  
  // Set velocity scaling
  xml << indent_str << "<SetVelocityScaling factor=\"" << config.velocity_scaling << "\"/>\n";
  
  // Set acceleration scaling
  xml << indent_str << "<SetAccelerationScaling factor=\"" << config.acceleration_scaling << "\"/>\n";
  
  xml << indent_str << "\n";
  
  return xml.str();
}

std::string XmlGenerator::generateSequence(const ProjectConfig& config, int ind) const
{
  std::ostringstream xml;
  std::string indent_str(ind * 2, ' ');
  
  xml << indent_str << "<!-- Motion sequence -->\n";
  
  for (const auto& motion : config.motions) {
    xml << motion->toXml(ind);
  }
  
  return xml.str();
}

}  // namespace ar_projects
