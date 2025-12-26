#ifndef AR_UI__BT_MANAGER_WINDOW_H_
#define AR_UI__BT_MANAGER_WINDOW_H_

#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QListWidget>
#include <QPushButton>
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

namespace ar_ui
{

class BTManagerWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit BTManagerWindow(QWidget* parent = nullptr);
  ~BTManagerWindow();

  void setRosNode(rclcpp::Node::SharedPtr node);

protected:
  void closeEvent(QCloseEvent* event) override;

private slots:
  void onRefreshProjects();
  void onProjectSelected(int index);
  void onGenerateXml();
  void onOpenInGroot();
  void onRunProject();
  void onStopProject();
  void onOpenXmlFile();
  void onSaveXml();
  void onClearLog();
  void onAbout();
  void processRosEvents();

private:
  void setupUi();
  void setupMenuBar();
  void updateProjectList();
  QString generateProjectXml(const QString& project_path);
  void executeTreeFile(const QString& tree_file);
  void startBtServer();
  void appendLog(const QString& msg, const QString& color = "white");

  // Widgets
  QWidget* central_widget_;
  QSplitter* main_splitter_;
  
  // Left panel - Projects
  QGroupBox* project_group_;
  QComboBox* project_combo_;
  QPushButton* refresh_btn_;
  QListWidget* project_info_list_;
  
  // Center panel - XML Editor
  QGroupBox* xml_group_;
  QTextEdit* xml_editor_;
  
  // Right panel - Execution
  QGroupBox* exec_group_;
  QLabel* status_label_;
  QPushButton* run_btn_;
  QPushButton* stop_btn_;
  QPushButton* groot_btn_;
  QTextEdit* log_text_;
  
  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr execution_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  QTimer* ros_timer_;
  
  // Processes
  QProcess* groot_process_;
  QProcess* server_process_;
  
  // State
  QString current_xml_file_;
  QString current_project_path_;
  QString groot_path_;
  QString pending_tree_file_;
  bool server_ready_;
};

}  // namespace ar_ui

#endif  // AR_UI__BT_MANAGER_WINDOW_H_
