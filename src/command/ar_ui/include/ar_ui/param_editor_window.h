#ifndef AR_UI__PARAM_EDITOR_WINDOW_H_
#define AR_UI__PARAM_EDITOR_WINDOW_H_

#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QSpinBox>
#include <QCheckBox>
#include <QGroupBox>
#include <QTextEdit>
#include <QThread>
#include <QTimer>
#include <QProcess>
#include <QTabWidget>
#include <QTableWidget>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <map>
#include <vector>

namespace ar_ui
{

/**
 * @brief Worker for ROS 2 operations to keep GUI responsive
 * 
 * Runs ROS2 spinning in a separate thread to prevent GUI blocking.
 */
class RosWorker : public QObject
{
  Q_OBJECT
public:
  /**
   * @brief Constructor
   * @param node ROS2 node to use
   */
  explicit RosWorker(rclcpp::Node::SharedPtr node);
  
  /**
   * @brief Stop the worker thread
   */
  void stop();
public Q_SLOTS:
  /**
   * @brief Spin ROS2 events (runs in thread)
   */
  void spin();
  
  /**
   * @brief Publish parameters to hardware interface
   * @param yaml YAML parameter string
   */
  void publishParams(const QString& yaml);
signals:
  /**
   * @brief Emitted when worker finishes
   */
  void finished();
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  std::atomic<bool> running_;
};

/**
 * @brief Standalone Window for parameter editing and system bringup
 */
class ParamEditorWindow : public QMainWindow
{
  Q_OBJECT

public:
  /**
   * @brief Constructor
   * @param parent Parent widget
   */
  explicit ParamEditorWindow(QWidget* parent = nullptr);
  
  /**
   * @brief Destructor
   */
  ~ParamEditorWindow() override;

public Q_SLOTS:
  /**
   * @brief Load parameters to hardware interface
   */
  void onLoadParams();
  
  /**
   * @brief Run robot bringup (real hardware)
   */
  void onRunRobot();
  
  /**
   * @brief Run robot bringup (simulation)
   */
  void onRunSimulation();
  
  /**
   * @brief Reload parameters from YAML file
   */
  void onReloadFromFile();
  
  /**
   * @brief Launch MoveIt
   */
  void onLaunchMoveIt();
  
  /**
   * @brief Launch RViz
   */
  void onLaunchRViz();

Q_SIGNALS:
  /**
   * @brief Signal to request parameter publishing
   * @param yaml YAML parameter string
   */
  void publishRequested(const QString& yaml);

private:
  /**
   * @brief Setup UI widgets and layout
   */
  void setupUi();
  
  /**
   * @brief Setup ROS2 node and worker thread
   */
  void setupRos();
  
  /**
   * @brief Load YAML configuration into UI widgets
   */
  void loadYamlToUi();
  
  /**
   * @brief Append message to log with color
   * @param msg Message to append
   * @param color HTML color (default: "#d4d4d4")
   */
  void appendLog(const QString& msg, const QString& color = "#d4d4d4");
  
  /**
   * @brief Buffer log message for later display
   * @param target Target text edit widget
   * @param msg Message to buffer
   */
  void bufferLog(QTextEdit* target, const QString& msg);
  
  /**
   * @brief Generate YAML from UI widget values
   * @return YAML string
   */
  QString generateYamlFromUi();
  
  /**
   * @brief Save YAML content to file
   * @param yaml_content YAML content to save
   */
  void saveYamlToFile(const QString& yaml_content);
  
  /**
   * @brief Check if a process is running
   * @param pattern Process name pattern to search for
   * @return true if process is running
   */
  bool isProcessRunning(const QString& pattern) const;
  
  /**
   * @brief Validate port IDs for real robot configuration
   * @param error_msg Output error message if validation fails
   * @return true if valid
   */
  bool validatePortIdsForRealRobot(QString& error_msg) const;
  
  /**
   * @brief Validate dual-axis driver configuration
   * @param error_msg Output error message if validation fails
   * @return true if valid
   */
  bool validateDualAxisDrivers(QString& error_msg) const;
  
  /**
   * @brief Kill all ROS processes (bringup, MoveIt, RViz)
   */
  void killAllRosProcesses();

  // ROS 2 Infrastructure
  rclcpp::Node::SharedPtr node_;
  QThread* ros_thread_;
  RosWorker* ros_worker_;
  
  // UI Components
  QWidget* central_widget_;
  QVBoxLayout* main_layout_;
  QTabWidget* tab_widget_;
  
  // Settings tab
  QLineEdit* robot_desc_edit_;
  QLineEdit* port_name_edit_;
  QSpinBox* driver_mode_spin_;
  QCheckBox* use_torque_offset_check_;
  
  // Joints table
  QTableWidget* joints_table_;
  
  // Action buttons
  QPushButton* reload_btn_;
  QPushButton* load_params_btn_;
  QPushButton* run_robot_btn_;
  QPushButton* run_simulation_btn_;
  QPushButton* launch_moveit_btn_;
  QPushButton* launch_rviz_btn_;
  QPushButton* stop_process_btn_;
  
  // Status
  QLabel* status_label_;
  
  // Log tabs
  QTabWidget* log_tabs_;
  QTextEdit* bringup_log_;
  QTextEdit* moveit_log_;
  QTextEdit* rviz_log_;
  
  // Logging infrastructure
  QMap<QTextEdit*, QStringList> log_buffers_;
  QTimer* log_flush_timer_;
  
  // Processes
  QProcess* bringup_process_;
  QProcess* moveit_process_;
  QProcess* rviz_process_;
  
  // State
  bool bringup_complete_;
  
private Q_SLOTS:
  /**
   * @brief Flush buffered log messages to display
   */
  void onFlushLogs();
  
  /**
   * @brief Stop all running processes
   */
  void onStopProcess();
};

}  // namespace ar_ui

#endif  // AR_UI__PARAM_EDITOR_WINDOW_H_
