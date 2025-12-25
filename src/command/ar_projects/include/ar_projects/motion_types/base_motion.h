#ifndef AR_PROJECTS__MOTION_TYPES__BASE_MOTION_H_
#define AR_PROJECTS__MOTION_TYPES__BASE_MOTION_H_

#include <string>
#include <vector>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace ar_projects
{

/**
 * @brief Base class for all motion types
 * Each motion type knows how to generate its XML representation
 */
class BaseMotion
{
public:
  BaseMotion(const std::string& type, const std::string& name = "");
  virtual ~BaseMotion() = default;

  /**
   * @brief Generate XML string for this motion
   * @param indent Indentation level for formatting
   * @return XML string representation
   */
  virtual std::string toXml(int indent = 0) const = 0;

  /**
   * @brief Parse motion from YAML node
   * @param node YAML node containing motion definition
   * @return true if parsing succeeded
   */
  virtual bool fromYaml(const YAML::Node& node) = 0;

  /**
   * @brief Get motion type identifier
   */
  const std::string& getType() const { return type_; }

  /**
   * @brief Get motion name
   */
  const std::string& getName() const { return name_; }

  /**
   * @brief Set motion name
   */
  void setName(const std::string& name) { name_ = name; }

protected:
  /**
   * @brief Helper to create indentation string
   */
  std::string indent(int level) const;

  /**
   * @brief Helper to format joint values as semicolon-separated string
   */
  std::string jointsToString(const std::vector<double>& joints) const;

  std::string type_;
  std::string name_;
};

/**
 * @brief Factory function type for creating motion instances
 */
using MotionFactory = std::function<std::shared_ptr<BaseMotion>()>;

/**
 * @brief Registry for motion type factories
 */
class MotionRegistry
{
public:
  static MotionRegistry& instance();

  /**
   * @brief Register a motion type factory
   */
  void registerMotion(const std::string& type, MotionFactory factory);

  /**
   * @brief Create a motion instance by type
   */
  std::shared_ptr<BaseMotion> create(const std::string& type) const;

  /**
   * @brief Get list of registered motion types
   */
  std::vector<std::string> getTypes() const;

private:
  MotionRegistry() = default;
  std::map<std::string, MotionFactory> factories_;
};

/**
 * @brief Helper macro for registering motion types
 */
#define REGISTER_MOTION_TYPE(TypeName, ClassName) \
  static bool _registered_##ClassName = []() { \
    MotionRegistry::instance().registerMotion(TypeName, []() { \
      return std::make_shared<ClassName>(); \
    }); \
    return true; \
  }()

}  // namespace ar_projects

#endif  // AR_PROJECTS__MOTION_TYPES__BASE_MOTION_H_
