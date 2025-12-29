#include "ar_ui/behavior_tree_panel.h"

#include <QDesktopServices>
#include <QUrl>
#include <QDir>
#include <QFile>
#include <QMessageBox>
#include <rviz_common/display_context.hpp>

namespace ar_ui
{

BehaviorTreePanel::BehaviorTreePanel(QWidget* parent)
  : rviz_common::Panel(parent)
  , groot_process_(nullptr)
  , server_process_(nullptr)
  , manager_process_(nullptr)
{
  setupUi();
}

BehaviorTreePanel::~BehaviorTreePanel()
{
  // Don't kill manager process - let it run independently
  if (groot_process_ && groot_process_->state() == QProcess::Running) {
    groot_process_->terminate();
  }
}

void BehaviorTreePanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  
  // Find Groot2
  QStringList groot_paths = {
    "/usr/local/bin/groot2",
    QDir::homePath() + "/Groot2/bin/groot2",
    QDir::homePath() + "/Groot2.AppImage",
    "/opt/Groot2/bin/groot2",
    "/usr/bin/groot2"
  };
  
  groot_path_ = "";
  for (const QString& path : groot_paths) {
    if (QFile::exists(path)) {
      groot_path_ = path;
      break;
    }
  }
  
  RCLCPP_INFO(node_->get_logger(), "BehaviorTreePanel initialized");
}

void BehaviorTreePanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void BehaviorTreePanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
}

void BehaviorTreePanel::setupUi()
{
  main_layout_ = new QVBoxLayout(this);
  main_layout_->setContentsMargins(5, 5, 5, 5);
  main_layout_->setSpacing(8);

  // Title
  QLabel* title = new QLabel("BehaviorTree", this);
  title->setStyleSheet("font-weight: bold; font-size: 12px;");
  title->setAlignment(Qt::AlignCenter);
  main_layout_->addWidget(title);

  // Open Manager button
  manager_btn_ = new QPushButton("🚀 Open Manager", this);
  manager_btn_->setStyleSheet(
    "QPushButton { background-color: #0e639c; color: white; font-weight: bold; "
    "padding: 10px; border-radius: 5px; }"
    "QPushButton:hover { background-color: #1177bb; }");
  manager_btn_->setToolTip("Open BehaviorTree Manager window");
  main_layout_->addWidget(manager_btn_);

  // Open Groot button
  groot_btn_ = new QPushButton("🌳 Open Groot2", this);
  groot_btn_->setStyleSheet(
    "QPushButton { background-color: #9c27b0; color: white; font-weight: bold; "
    "padding: 10px; border-radius: 5px; }"
    "QPushButton:hover { background-color: #ab47bc; }");
  groot_btn_->setToolTip("Open Groot2 BehaviorTree editor");
  main_layout_->addWidget(groot_btn_);

  // Status label
  status_label_ = new QLabel("Ready", this);
  status_label_->setStyleSheet("color: gray; font-size: 10px;");
  status_label_->setAlignment(Qt::AlignCenter);
  main_layout_->addWidget(status_label_);

  main_layout_->addStretch();

  // Initialize unused members
  project_combo_ = nullptr;
  refresh_btn_ = nullptr;
  generate_btn_ = nullptr;
  run_btn_ = nullptr;
  stop_btn_ = nullptr;
  log_list_ = nullptr;
  server_status_label_ = nullptr;

  // Connections
  connect(manager_btn_, &QPushButton::clicked, this, &BehaviorTreePanel::onOpenManager);
  connect(groot_btn_, &QPushButton::clicked, this, &BehaviorTreePanel::onOpenInGroot);
}

void BehaviorTreePanel::onOpenManager()
{
  if (!manager_process_) {
    manager_process_ = new QProcess(this);
  }
  
  if (manager_process_->state() != QProcess::Running) {
    // Launch bt_manager executable
    manager_process_->start("ros2", QStringList() << "run" << "ar_ui" << "bt_manager");
    status_label_->setText("Manager opened");
    RCLCPP_INFO(node_->get_logger(), "Launching BT Manager");
  } else {
    status_label_->setText("Manager running");
  }
}

void BehaviorTreePanel::onOpenInGroot()
{
  if (!groot_path_.isEmpty() && QFile::exists(groot_path_)) {
    if (!groot_process_) {
      groot_process_ = new QProcess(this);
    }
    
    if (groot_process_->state() != QProcess::Running) {
      groot_process_->start(groot_path_, QStringList());
      status_label_->setText("Groot2 opened");
      RCLCPP_INFO(node_->get_logger(), "Launching Groot2");
    } else {
      status_label_->setText("Groot2 running");
    }
  } else {
    QMessageBox::warning(this, "Groot Not Found",
      "Groot2 not installed.\n\nDownload from:\nhttps://www.behaviortree.dev/groot");
  }
}

// Empty stubs for unused functions
void BehaviorTreePanel::updateProjectList() {}
void BehaviorTreePanel::onRefreshProjects() {}
void BehaviorTreePanel::onProjectSelected(int) {}
QString BehaviorTreePanel::generateProjectXml(const QString&) { return ""; }
void BehaviorTreePanel::onGenerateXml() {}
void BehaviorTreePanel::startServer() {}
void BehaviorTreePanel::onServerStarted() {}
void BehaviorTreePanel::onServerError() {}
void BehaviorTreePanel::onServerOutput() {}
void BehaviorTreePanel::ensureServerRunning() {}
void BehaviorTreePanel::onRunProject() {}
void BehaviorTreePanel::executeTreeFile(const QString&) {}
void BehaviorTreePanel::onStop() {}

}  // namespace ar_ui

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ar_ui::BehaviorTreePanel, rviz_common::Panel)
