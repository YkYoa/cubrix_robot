#include "ar_projects/motion_types/base_motion.h"
#include <sstream>
#include <iomanip>

namespace ar_projects
{

BaseMotion::BaseMotion(const std::string& type, const std::string& name)
  : type_(type), name_(name)
{
}

std::string BaseMotion::indent(int level) const
{
  return std::string(level * 2, ' ');
}

std::string BaseMotion::jointsToString(const std::vector<double>& joints) const
{
  std::ostringstream oss;
  for (size_t i = 0; i < joints.size(); ++i) {
    if (i > 0) oss << ";";
    oss << std::fixed << std::setprecision(4) << joints[i];
  }
  return oss.str();
}

// Motion Registry implementation

MotionRegistry& MotionRegistry::instance()
{
  static MotionRegistry registry;
  return registry;
}

void MotionRegistry::registerMotion(const std::string& type, MotionFactory factory)
{
  factories_[type] = factory;
}

std::shared_ptr<BaseMotion> MotionRegistry::create(const std::string& type) const
{
  auto it = factories_.find(type);
  if (it != factories_.end()) {
    return it->second();
  }
  return nullptr;
}

std::vector<std::string> MotionRegistry::getTypes() const
{
  std::vector<std::string> types;
  for (const auto& pair : factories_) {
    types.push_back(pair.first);
  }
  return types;
}

}  // namespace ar_projects
