#ifndef AR_UI__BT_MANAGER_WINDOW_H_
#define AR_UI__BT_MANAGER_WINDOW_H_

#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QListWidget>
#include <QPushButton>
#include <QCheckBox>
#include <QSpinBox>
#include <QLabel>
#include <QComboBox>
#include <QTextEdit>
#include <QGroupBox>
#include <QSplitter>
#include <QStatusBar>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QTimer>
#include <QProcess>
#include <QFileDialog>
#include <QMessageBox>
#include <QCloseEvent>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "ar_projects/motion_executor.h"

namespace ar_ui
{

/**
 * @brief Main window for BehaviorTree project management
 * 
 * Qt-based window for managing YAML projects, generating XML trees,
 * and executing BehaviorTree sequences. Integrates with Groot2 for
 * visual editing and provides real-time execution monitoring.
 */
class BTManagerWindow : public QMainWindow
{
  Q_OBJECT

public:
  /**
   * @brief Constructor
   * @param parent Parent widget
   */
  explicit BTManagerWindow(QWidget* parent = nullptr);
  
  /**
   * @brief Destructor
   */
  ~BTManagerWindow();

  /**
   * @brief Set ROS2 node for communication
   * @param node ROS2 node pointer
   */
  void setRosNode(rclcpp::Node::SharedPtr node);

protected:
  /**
   * @brief Handle window close event
   * @param event Close event
   */
  void closeEvent(QCloseEvent* event) override;

private slots:
  /**
   * @brief Refresh project list from disk
   */
  void onRefreshProjects();
  
  /**
   * @brief Handle project selection from dropdown
   * @param index Selected project index
   */
  void onProjectSelected(int index);
  
  /**
   * @brief Generate XML from selected project
   */
  void onGenerateXml();
  
  /**
   * @brief Generate YAML from current XML
   */
  void onGenerateYaml();
  
  /**
   * @brief Open current tree in Groot2 editor
   */
  void onOpenInGroot();
  
  /**
   * @brief Run the selected project
   */
  void onRunProject();
  
  /**
   * @brief Stop current execution
   */
  void onStopProject();
  
  /**
   * @brief Open XML file from disk
   */
  void onOpenXmlFile();
  
  /**
   * @brief Reload YAML configuration
   */
  void onReloadYaml();
  
  /**
   * @brief Save XML to file
   */
  void onSaveXml();
  
  /**
   * @brief Clear execution log
   */
  void onClearLog();
  
  /**
   * @brief Get current robot state
   */
  void onGetRobotState();
  
  /**
   * @brief Run code-based task from project
   */
  void onRunTask();
  
  /**
   * @brief Show about dialog
   */
  void onAbout();
  
  /**
   * @brief Process ROS2 events (called by timer)
   */
  void processRosEvents();

private:
  /**
   * @brief Setup UI widgets and layout
   */
  void setupUi();
  
  /**
   * @brief Setup menu bar
   */
  void setupMenuBar();
  
  /**
   * @brief Update project list from disk
   */
  void updateProjectList();
  
  /**
   * @brief Generate XML from project YAML
   * @param project_path Path to project YAML file
   * @return Generated XML string
   */
  QString generateProjectXml(const QString& project_path);
  
  /**
   * @brief Execute a BehaviorTree XML file
   * @param tree_file Path to XML file
   */
  void executeTreeFile(const QString& tree_file);
  
  /**
   * @brief Start BT server node if not running
   */
  void startBtServer();
  
  /**
   * @brief Append message to log with color
   * @param msg Message to append
   * @param color HTML color (default: "white")
   */
  void appendLog(const QString& msg, const QString& color = "white");

  // Widgets
  QWidget* central_widget_;
  QSplitter* main_splitter_;
  
  // Left panel - Projects
  QGroupBox* project_group_;
  QComboBox* project_combo_;
  QPushButton* refresh_btn_;
  QListWidget* sequence_list_;
  
  // Center panel - XML Editor
  QGroupBox* xml_group_;
  QTextEdit* xml_editor_;
  
  // Right panel - Execution
  QGroupBox* exec_group_;
  QLabel* status_label_;
  QPushButton* run_btn_;
  QPushButton* stop_btn_;
  QPushButton* groot_btn_;
  QLabel* task_info_label_;       // Show task name if configured
  QCheckBox* loop_checkbox_;
  QSpinBox* loop_count_spinbox_;
  QTextEdit* log_text_;
  
  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr execution_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  sensor_msgs::msg::JointState latest_joint_state_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  QTimer* ros_timer_;
  
  // Processes
  QProcess* groot_process_;
  QProcess* server_process_;
  
  // State
  QString current_xml_file_;
  QString current_project_path_;
  QString current_task_name_;     // NEW: Current project's task
  QString groot_path_;
  QString pending_tree_file_;
  bool server_ready_;
  int current_loop_;
  int target_loops_;
  std::shared_ptr<ar_projects::MotionExecutor> active_executor_;
  bool stop_requested_;
};

}  // namespace ar_ui

#endif  // AR_UI__BT_MANAGER_WINDOW_H_
