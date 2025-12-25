#ifndef AR_UI__BEHAVIOR_TREE_PANEL_H_
#define AR_UI__BEHAVIOR_TREE_PANEL_H_

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QListWidget>
#include <QGroupBox>
#include <QProcess>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

namespace ar_ui
{

/**
 * @brief RViz Panel for Behavior Tree project management
 * 
 * Features:
 * - Project selector from ar_projects
 * - Generate XML from YAML  
 * - Open generated XML in Groot2
 * - Execute behavior trees (auto-starts bt_server_node)
 */
class BehaviorTreePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit BehaviorTreePanel(QWidget* parent = nullptr);
  ~BehaviorTreePanel() override;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config& config) override;

public Q_SLOTS:
  void onRefreshProjects();
  void onProjectSelected(int index);
  void onGenerateXml();
  void onOpenInGroot();
  void onRunProject();
  void onStop();

private Q_SLOTS:
  void onServerStarted();
  void onServerError();
  void onServerOutput();

private:
  void setupUi();
  void updateProjectList();
  void executeTreeFile(const QString& tree_file);
  QString generateProjectXml(const QString& project_path);
  void ensureServerRunning();
  void startServer();

  // ROS 2 node
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr execution_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;

  // UI Components
  QVBoxLayout* main_layout_;
  
  // Project selection
  QComboBox* project_combo_;
  QPushButton* refresh_btn_;
  
  // Actions
  QPushButton* generate_btn_;
  QPushButton* groot_btn_;
  QPushButton* run_btn_;
  QPushButton* stop_btn_;
  
  // Status
  QLabel* status_label_;
  QLabel* server_status_label_;
  QListWidget* log_list_;

  // State
  QString current_xml_file_;
  QString groot_path_;
  QProcess* groot_process_;
  QProcess* server_process_;
  QString pending_tree_file_;  // Tree to execute after server starts
};

}  // namespace ar_ui

#endif  // AR_UI__BEHAVIOR_TREE_PANEL_H_
