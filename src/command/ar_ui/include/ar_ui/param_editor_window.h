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
 */
class RosWorker : public QObject
{
  Q_OBJECT
public:
  explicit RosWorker(rclcpp::Node::SharedPtr node);
  void stop();
public Q_SLOTS:
  void spin();
  void publishParams(const QString& yaml);
signals:
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
  explicit ParamEditorWindow(QWidget* parent = nullptr);
  ~ParamEditorWindow() override;

public Q_SLOTS:
  void onLoadParams();
  void onRunRobot();
  void onRunSimulation();
  void onReloadFromFile();
  void onLaunchMoveIt();
  void onLaunchRViz();

Q_SIGNALS:
  void publishRequested(const QString& yaml);

private:
  void setupUi();
  void setupRos();
  void loadYamlToUi();
  void appendLog(const QString& msg, const QString& color = "#d4d4d4");
  void bufferLog(QTextEdit* target, const QString& msg);
  QString generateYamlFromUi();
  void saveYamlToFile(const QString& yaml_content);
  bool isProcessRunning(const QString& pattern) const;
  bool validatePortIdsForRealRobot(QString& error_msg) const;
  bool validateDualAxisDrivers(QString& error_msg) const;
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
  void onFlushLogs();
  void onStopProcess();
};

}  // namespace ar_ui

#endif  // AR_UI__PARAM_EDITOR_WINDOW_H_
