#ifndef AR_PROJECTS__TASKS__TASK_BASE_H_
#define AR_PROJECTS__TASKS__TASK_BASE_H_

#include <string>
#include <memory>
#include <map>
#include <functional>
#include <yaml-cpp/yaml.h>

namespace ar_projects
{

// Forward declaration
class MotionExecutor;

/**
 * @brief Base class for all code-based tasks
 * 
 * Tasks allow complex logic (loops, calculations) that can't be 
 * expressed in declarative YAML configs.
 * 
 * Example usage:
 *   class DrawRectangleTask : public TaskBase {
 *     std::string name() const override { return "draw_rectangle"; }
 *     bool execute(MotionExecutor& executor) override { ... }
 *   };
 */
class TaskBase
{
public:
  virtual ~TaskBase() = default;

  /**
   * @brief Get task identifier name
   * @return Unique task name (e.g., "draw_rectangle")
   */
  virtual std::string name() const = 0;

  /**
   * @brief Configure task from YAML parameters
   * @param params YAML node containing task_params from config
   * @return true if configuration succeeded
   */
  virtual bool configure(const YAML::Node& params) { (void)params; return true; }

  /**
   * @brief Execute the task
   * @param executor Motion executor for commanding robot
   * @return true if execution succeeded
   */
  virtual bool execute(MotionExecutor& executor) = 0;

  /**
   * @brief Get description of what this task does
   * @return Human-readable description
   */
  virtual std::string description() const { return ""; }
};

/**
 * @brief Factory function type for creating task instances
 */
using TaskFactory = std::function<std::shared_ptr<TaskBase>()>;

/**
 * @brief Registry for discovering and creating tasks by name
 * 
 * Singleton pattern - use TaskRegistry::instance() to access.
 */
class TaskRegistry
{
public:
  /**
   * @brief Get singleton instance
   */
  static TaskRegistry& instance();

  /**
   * @brief Register a task factory
   * @param name Task name identifier
   * @param factory Function that creates task instances
   */
  void registerTask(const std::string& name, TaskFactory factory);

  /**
   * @brief Create a task instance by name
   * @param name Task name to create
   * @return Task instance or nullptr if not found
   */
  std::shared_ptr<TaskBase> create(const std::string& name) const;

  /**
   * @brief Check if a task is registered
   * @param name Task name to check
   * @return true if task exists in registry
   */
  bool hasTask(const std::string& name) const;

  /**
   * @brief Get list of all registered task names
   * @return Vector of task names
   */
  std::vector<std::string> getTaskNames() const;

private:
  TaskRegistry() = default;
  std::map<std::string, TaskFactory> factories_;
};

/**
 * @brief Helper macro for registering task types
 * 
 * Usage:
 *   REGISTER_TASK("draw_rectangle", DrawRectangleTask);
 * 
 * This registers the task at static initialization time.
 */
#define REGISTER_TASK(TaskName, ClassName) \
  static bool _registered_task_##ClassName = []() { \
    ar_projects::TaskRegistry::instance().registerTask(TaskName, []() { \
      return std::make_shared<ClassName>(); \
    }); \
    return true; \
  }()

}  // namespace ar_projects

#endif  // AR_PROJECTS__TASKS__TASK_BASE_H_
