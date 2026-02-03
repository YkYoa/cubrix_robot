#include "ar_projects/tasks/task_base.h"

namespace ar_projects
{

TaskRegistry& TaskRegistry::instance()
{
  static TaskRegistry registry;
  return registry;
}

void TaskRegistry::registerTask(const std::string& name, TaskFactory factory)
{
  factories_[name] = factory;
}

std::shared_ptr<TaskBase> TaskRegistry::create(const std::string& name) const
{
  auto it = factories_.find(name);
  if (it != factories_.end()) {
    return it->second();
  }
  return nullptr;
}

bool TaskRegistry::hasTask(const std::string& name) const
{
  return factories_.find(name) != factories_.end();
}

std::vector<std::string> TaskRegistry::getTaskNames() const
{
  std::vector<std::string> names;
  names.reserve(factories_.size());
  for (const auto& pair : factories_) {
    names.push_back(pair.first);
  }
  return names;
}

}  // namespace ar_projects
