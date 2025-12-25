#include "ar_ui/behavior_tree_panel.h"
#include <rviz_common/display_context.hpp>

namespace ar_ui
{

BehaviorTreePanel::BehaviorTreePanel(QWidget* parent)
  : rviz_common::Panel(parent)
  , groot_process_(nullptr)
  , server_process_(nullptr)
{
  setupUi();
}

BehaviorTreePanel::~BehaviorTreePanel() {}

void BehaviorTreePanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  RCLCPP_INFO(node_->get_logger(), "Panel initialized");
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
  main_layout_->setSpacing(5);

  // Compact project selection
  project_combo_ = new QComboBox(this);
  project_combo_->addItem("-- Select Project --");
  project_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  main_layout_->addWidget(project_combo_);

  // Compact buttons in single row
  QHBoxLayout* btn_layout = new QHBoxLayout();
  btn_layout->setSpacing(3);
  
  run_btn_ = new QPushButton("▶", this);
  run_btn_->setFixedSize(30, 25);
  run_btn_->setToolTip("Run Project");
  
  stop_btn_ = new QPushButton("■", this);
  stop_btn_->setFixedSize(30, 25);
  stop_btn_->setToolTip("Stop");
  
  groot_btn_ = new QPushButton("🌳", this);
  groot_btn_->setFixedSize(30, 25);
  groot_btn_->setToolTip("Open in Groot");
  
  refresh_btn_ = new QPushButton("↻", this);
  refresh_btn_->setFixedSize(30, 25);
  refresh_btn_->setToolTip("Refresh");
  
  btn_layout->addWidget(run_btn_);
  btn_layout->addWidget(stop_btn_);
  btn_layout->addWidget(groot_btn_);
  btn_layout->addWidget(refresh_btn_);
  btn_layout->addStretch();
  main_layout_->addLayout(btn_layout);

  // Status label
  status_label_ = new QLabel("Ready", this);
  status_label_->setStyleSheet("QLabel { font-size: 10px; color: gray; }");
  main_layout_->addWidget(status_label_);

  // Compact log
  log_list_ = new QListWidget(this);
  log_list_->setMaximumHeight(60);
  log_list_->setStyleSheet("QListWidget { font-size: 9px; }");
  main_layout_->addWidget(log_list_);

  // Set maximum width to keep panel compact
  setMaximumWidth(250);
  
  // Initialize unused pointers
  server_status_label_ = nullptr;
  generate_btn_ = nullptr;
}

// Empty stubs
void BehaviorTreePanel::updateProjectList() {}
void BehaviorTreePanel::onRefreshProjects() {}
void BehaviorTreePanel::onProjectSelected(int) {}
QString BehaviorTreePanel::generateProjectXml(const QString&) { return ""; }
void BehaviorTreePanel::onGenerateXml() {}
void BehaviorTreePanel::onOpenInGroot() {}
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
