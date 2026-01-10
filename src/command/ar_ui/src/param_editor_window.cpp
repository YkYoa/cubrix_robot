#include "ar_ui/param_editor_window.h"

#include <QFile>
#include <QTextStream>
#include <QScrollBar>
#include <QTime>
#include <QHeaderView>
#include <QFontDatabase>
#include <QMessageBox>
#include <QMenuBar>
#include <QStatusBar>
#include <QRegularExpression>
#include <QMap>
#include <QDir>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <QApplication>
#include <QCoreApplication>

#include <csignal>
#include <unistd.h>
#include <sys/types.h>

namespace ar_ui
{

RosWorker::RosWorker(rclcpp::Node::SharedPtr node)
  : node_(node), running_(true)
{
  pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/ar_params/loaded", rclcpp::QoS(10).transient_local());
}

void RosWorker::spin()
{
  // Use spin_some in a loop to allow Qt signals to be processed
  while (running_ && rclcpp::ok()) {
    rclcpp::spin_some(node_);
    QThread::msleep(10);  // Allow Qt event loop to process
    QCoreApplication::processEvents();  // Process pending Qt events (signals)
  }
  emit finished();
}

void RosWorker::stop()
{
  running_ = false;
}

void RosWorker::publishParams(const QString& yaml)
{
  auto msg = std_msgs::msg::String();
  msg.data = yaml.toStdString();
  pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "Published %zu bytes to /ar_params/loaded", msg.data.size());
}

ParamEditorWindow::ParamEditorWindow(QWidget* parent)
  : QMainWindow(parent)
  , bringup_process_(nullptr)
  , moveit_process_(nullptr)
  , rviz_process_(nullptr)
  , bringup_complete_(false)
{
  // Initialize ROS node
  node_ = rclcpp::Node::make_shared("param_editor_ui");
  
  // Setup Log flush timer (throttle UI updates to 10Hz)
  log_flush_timer_ = new QTimer(this);
  connect(log_flush_timer_, &QTimer::timeout, this, &ParamEditorWindow::onFlushLogs);
  log_flush_timer_->start(100);
  
  setupUi();
  setupRos(); // Initialize the ROS thread and worker
  loadYamlToUi();
  
  RCLCPP_INFO(node_->get_logger(), "ParamEditorWindow started");
}

void ParamEditorWindow::setupRos()
{
  ros_thread_ = new QThread(this);
  ros_worker_ = new RosWorker(node_);
  ros_worker_->moveToThread(ros_thread_);
  
  // Connections
  connect(ros_thread_, &QThread::started, ros_worker_, &RosWorker::spin);
  connect(ros_worker_, &RosWorker::finished, ros_thread_, &QThread::quit);
  connect(ros_worker_, &RosWorker::finished, ros_worker_, &RosWorker::deleteLater);
  connect(ros_thread_, &QThread::finished, ros_thread_, &QThread::deleteLater);
  
  // Publish signal
  connect(this, &ParamEditorWindow::publishRequested, ros_worker_, &RosWorker::publishParams);
  
  ros_thread_->start();
}

ParamEditorWindow::~ParamEditorWindow()
{
  if (ros_thread_ && ros_thread_->isRunning()) {
    if (ros_worker_) {
      ros_worker_->stop();  // Signal the spin loop to stop
    }
    rclcpp::shutdown();
    ros_thread_->quit();
    ros_thread_->wait(1000);
  }
  
  // Helper to kill entire process group, not just the shell
  auto terminateProcessGroup = [](QProcess* p) {
    if (p && p->state() == QProcess::Running) {
      qint64 pid = p->processId();
      if (pid > 0) {
        // Send SIGTERM to the entire process group
        ::kill(-pid, SIGTERM);
      }
      if (!p->waitForFinished(1000)) {
        // Force kill the process group if still running
        if (pid > 0) {
          ::kill(-pid, SIGKILL);
        }
        p->kill();
      }
    }
  };
  
  if (bringup_process_) terminateProcessGroup(bringup_process_);
  if (moveit_process_) terminateProcessGroup(moveit_process_);
  if (rviz_process_) terminateProcessGroup(rviz_process_);
}

void ParamEditorWindow::setupUi()
{
  setWindowTitle("AR Parameter Editor");
  setMinimumSize(600, 700);
  resize(700, 800);
  
  // Dark theme
  setStyleSheet(
    "QMainWindow { background-color: #1e1e1e; }"
    "QLabel { color: #d4d4d4; }"
    "QGroupBox { color: #d4d4d4; border: 1px solid #3c3c3c; margin-top: 10px; padding-top: 10px; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; }");
  
  central_widget_ = new QWidget(this);
  setCentralWidget(central_widget_);
  
  main_layout_ = new QVBoxLayout(central_widget_);
  main_layout_->setContentsMargins(12, 12, 12, 12);
  main_layout_->setSpacing(10);
  
  // Title
  QLabel* title = new QLabel("🔧 AR Hardware Parameters", this);
  title->setStyleSheet("font-weight: bold; font-size: 18px; color: #569cd6;");
  title->setAlignment(Qt::AlignCenter);
  main_layout_->addWidget(title);
  
  // Tab widget
  tab_widget_ = new QTabWidget(this);
  tab_widget_->setStyleSheet(
    "QTabWidget::pane { border: 1px solid #3c3c3c; background: #252526; padding: 10px; }"
    "QTabBar::tab { background: #1e1e1e; color: #ccc; padding: 8px 16px; margin-right: 2px; }"
    "QTabBar::tab:selected { background: #0e639c; color: white; }"
    "QTabBar::tab:hover { background: #2d2d2d; }");
  
  // ============ Settings Tab ============
  QWidget* settings_tab = new QWidget();
  QVBoxLayout* settings_layout = new QVBoxLayout(settings_tab);
  settings_layout->setSpacing(12);
  
  // Robot description group
  QGroupBox* robot_group = new QGroupBox("Robot", settings_tab);
  robot_group->setStyleSheet("QGroupBox { font-weight: bold; }");
  QGridLayout* robot_layout = new QGridLayout(robot_group);
  robot_layout->setSpacing(10);
  
  robot_layout->addWidget(new QLabel("Description:"), 0, 0);
  robot_desc_edit_ = new QLineEdit("ar", settings_tab);
  robot_desc_edit_->setToolTip("Robot description name (e.g., ar, ar6dof)");
  robot_desc_edit_->setStyleSheet("background: #1e1e1e; color: white; padding: 6px; border: 1px solid #3c3c3c;");
  robot_layout->addWidget(robot_desc_edit_, 0, 1);
  
  settings_layout->addWidget(robot_group);
  
  // Port settings group
  QGroupBox* port_group = new QGroupBox("Port Settings", settings_tab);
  port_group->setStyleSheet("QGroupBox { font-weight: bold; }");
  QGridLayout* port_layout = new QGridLayout(port_group);
  port_layout->setSpacing(10);
  
  port_layout->addWidget(new QLabel("Port Name:"), 0, 0);
  port_name_edit_ = new QLineEdit("enp2s0", settings_tab);
  port_name_edit_->setStyleSheet("background: #1e1e1e; color: white; padding: 6px; border: 1px solid #3c3c3c;");
  port_layout->addWidget(port_name_edit_, 0, 1);
  
  port_layout->addWidget(new QLabel("Driver Mode:"), 1, 0);
  driver_mode_spin_ = new QSpinBox(settings_tab);
  driver_mode_spin_->setRange(0, 1);
  driver_mode_spin_->setValue(1);
  driver_mode_spin_->setToolTip("0 = Profile Position, 1 = Cyclic Position");
  driver_mode_spin_->setStyleSheet("background: #1e1e1e; color: white; padding: 6px;");
  port_layout->addWidget(driver_mode_spin_, 1, 1);
  
  use_torque_offset_check_ = new QCheckBox("Use Torque Offset", settings_tab);
  use_torque_offset_check_->setStyleSheet("color: white;");
  port_layout->addWidget(use_torque_offset_check_, 2, 0, 1, 2);
  
  settings_layout->addWidget(port_group);
  settings_layout->addStretch();
  
  tab_widget_->addTab(settings_tab, "⚙️ Settings");
  
  // ============ Joints Tab ============
  QWidget* joints_tab = new QWidget();
  QVBoxLayout* joints_layout = new QVBoxLayout(joints_tab);
  
  joints_table_ = new QTableWidget(joints_tab);
  joints_table_->setColumnCount(7);
  joints_table_->setHorizontalHeaderLabels({"Joint", "Drive", "Port", "Gear Ratio", "Encoder Res", "Offset", "Log"});
  joints_table_->horizontalHeader()->setStretchLastSection(true);
  joints_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  joints_table_->setStyleSheet(
    "QTableWidget { background: #1e1e1e; color: white; gridline-color: #3c3c3c; }"
    "QHeaderView::section { background: #2d2d2d; color: white; padding: 6px; border: 1px solid #3c3c3c; font-weight: bold; }"
    "QTableWidget::item { padding: 4px; }"
    "QTableWidget::item:alternate { background: #252526; }");
  joints_table_->setAlternatingRowColors(true);
  
  joints_layout->addWidget(joints_table_);
  
  tab_widget_->addTab(joints_tab, "🦾 Joints");
  
  main_layout_->addWidget(tab_widget_, 1);
  
  // ============ Action Buttons ============
  QHBoxLayout* btn_row1 = new QHBoxLayout();
  
  reload_btn_ = new QPushButton("🔄 Reload from File", this);
  reload_btn_->setStyleSheet(
    "QPushButton { background: #555; color: white; padding: 10px 20px; border-radius: 5px; font-size: 12px; }"
    "QPushButton:hover { background: #666; }");
  btn_row1->addWidget(reload_btn_);
  
  load_params_btn_ = new QPushButton("📊 Load Parameters", this);
  load_params_btn_->setStyleSheet(
    "QPushButton { background: #4CAF50; color: white; font-weight: bold; padding: 10px 20px; border-radius: 5px; font-size: 12px; }"
    "QPushButton:hover { background: #66BB6A; }"
    "QPushButton:disabled { background: #555; }");
  btn_row1->addWidget(load_params_btn_);
  
  main_layout_->addLayout(btn_row1);
  
  QHBoxLayout* btn_row2 = new QHBoxLayout();
  
  run_robot_btn_ = new QPushButton("🤖 Run Robot", this);
  run_robot_btn_->setStyleSheet(
    "QPushButton { background: #FF5722; color: white; font-weight: bold; padding: 14px 30px; border-radius: 5px; font-size: 14px; }"
    "QPushButton:hover { background: #FF7043; }"
    "QPushButton:disabled { background: #555; }");
  btn_row2->addWidget(run_robot_btn_);
  
  run_simulation_btn_ = new QPushButton("🎮 Run Simulation", this);
  run_simulation_btn_->setStyleSheet(
    "QPushButton { background: #9C27B0; color: white; font-weight: bold; padding: 14px 30px; border-radius: 5px; font-size: 14px; }"
    "QPushButton:hover { background: #AB47BC; }"
    "QPushButton:disabled { background: #555; }");
  btn_row2->addWidget(run_simulation_btn_);
  
  main_layout_->addLayout(btn_row2);
  
  // Third button row - MoveIt and RViz
  QHBoxLayout* btn_row3 = new QHBoxLayout();
  
  launch_moveit_btn_ = new QPushButton("🦾 Launch MoveIt", this);
  launch_moveit_btn_->setStyleSheet(
    "QPushButton { background: #2196F3; color: white; font-weight: bold; padding: 10px 20px; border-radius: 5px; font-size: 12px; }"
    "QPushButton:hover { background: #42A5F5; }"
    "QPushButton:disabled { background: #555; }");
  btn_row3->addWidget(launch_moveit_btn_);
  
  launch_rviz_btn_ = new QPushButton("👁️ Launch RViz", this);
  launch_rviz_btn_->setStyleSheet(
    "QPushButton { background: #00BCD4; color: white; font-weight: bold; padding: 10px 20px; border-radius: 5px; font-size: 12px; }"
    "QPushButton:hover { background: #26C6DA; }"
    "QPushButton:disabled { background: #555; }");
  btn_row3->addWidget(launch_rviz_btn_);

  // Stop button
  stop_process_btn_ = new QPushButton("🛑 Stop Active Process", this);
  stop_process_btn_->setStyleSheet(
    "QPushButton { background: #D32F2F; color: white; font-weight: bold; padding: 10px 20px; border-radius: 5px; font-size: 12px; }"
    "QPushButton:hover { background: #E53935; }"
    "QPushButton:disabled { background: #555; }");
  btn_row3->addWidget(stop_process_btn_);
  
  main_layout_->addLayout(btn_row3);
  
  // Status
  status_label_ = new QLabel("⚪ Ready", this);
  status_label_->setStyleSheet("color: #808080; font-weight: bold; font-size: 12px; padding: 5px;");
  status_label_->setAlignment(Qt::AlignCenter);
  main_layout_->addWidget(status_label_);
  
  // Log tabs - separate log for each process
  QLabel* log_label = new QLabel("🖥️ Execution Logs:", this);
  log_label->setStyleSheet("font-weight: bold; color: #569cd6; font-size: 11px;");
  main_layout_->addWidget(log_label);
  
  log_tabs_ = new QTabWidget(this);
  log_tabs_->setStyleSheet(
    "QTabWidget::pane { border: 1px solid #3c3c3c; background: #0d1117; }"
    "QTabBar::tab { background: #1e1e1e; color: #808080; padding: 8px 16px; margin-right: 2px; }"
    "QTabBar::tab:selected { background: #0d1117; color: white; border-bottom: 2px solid #569cd6; }");
  
  QFont log_font = QFontDatabase::systemFont(QFontDatabase::FixedFont);
  log_font.setPointSize(10);
  
  // Bringup log
  bringup_log_ = new QTextEdit(this);
  bringup_log_->setReadOnly(true);
  bringup_log_->setUndoRedoEnabled(false);
  bringup_log_->document()->setMaximumBlockCount(3000);
  bringup_log_->setFont(log_font);
  bringup_log_->setStyleSheet("background-color: #0d1117; color: #d4d4d4; border: none; padding: 8px;");
  log_tabs_->addTab(bringup_log_, "🤖 Bringup");
  
  // MoveIt log
  moveit_log_ = new QTextEdit(this);
  moveit_log_->setReadOnly(true);
  moveit_log_->setUndoRedoEnabled(false);
  moveit_log_->document()->setMaximumBlockCount(3000);
  moveit_log_->setFont(log_font);
  moveit_log_->setStyleSheet("background-color: #0d1117; color: #d4d4d4; border: none; padding: 8px;");
  log_tabs_->addTab(moveit_log_, "🦾 MoveIt");
  
  // RViz log
  rviz_log_ = new QTextEdit(this);
  rviz_log_->setReadOnly(true);
  rviz_log_->setUndoRedoEnabled(false);
  rviz_log_->document()->setMaximumBlockCount(3000);
  rviz_log_->setFont(log_font);
  rviz_log_->setStyleSheet("background-color: #0d1117; color: #d4d4d4; border: none; padding: 8px;");
  log_tabs_->addTab(rviz_log_, "👁️ RViz");
  
  log_tabs_->setMinimumHeight(200);
  main_layout_->addWidget(log_tabs_, 1);  // stretch factor 1
  
  // Connections
  connect(reload_btn_, &QPushButton::clicked, this, &ParamEditorWindow::onReloadFromFile);
  connect(load_params_btn_, &QPushButton::clicked, this, &ParamEditorWindow::onLoadParams);
  connect(run_robot_btn_, &QPushButton::clicked, this, &ParamEditorWindow::onRunRobot);
  connect(run_simulation_btn_, &QPushButton::clicked, this, &ParamEditorWindow::onRunSimulation);
  connect(launch_moveit_btn_, &QPushButton::clicked, this, &ParamEditorWindow::onLaunchMoveIt);
  connect(launch_rviz_btn_, &QPushButton::clicked, this, &ParamEditorWindow::onLaunchRViz);
  connect(stop_process_btn_, &QPushButton::clicked, this, &ParamEditorWindow::onStopProcess);
  
  appendLog("🤖 Parameter Editor Ready", "#569cd6");
}

void ParamEditorWindow::loadYamlToUi()
{
  try {
    std::string config_dir = ament_index_cpp::get_package_share_directory("ar_control");
    std::string yaml_path = config_dir + "/config/ar_drive.yaml";
    
    YAML::Node config = YAML::LoadFile(yaml_path);
    
    if (config["port_info"] && config["port_info"]["port_name"]) {
      port_name_edit_->setText(QString::fromStdString(config["port_info"]["port_name"].as<std::string>()));
    }
    if (config["driver_info"] && config["driver_info"]["driver_mode"]) {
      driver_mode_spin_->setValue(config["driver_info"]["driver_mode"].as<int>());
    }
    if (config["use_torque_offset"]) {
      use_torque_offset_check_->setChecked(config["use_torque_offset"].as<bool>());
    }
    
    joints_table_->setRowCount(0);
    
    // Get port_ids map
    YAML::Node port_ids_node = config["port_ids"];
    
    if (config["drives"]) {
      for (const auto& drive_pair : config["drives"]) {
        QString drive_id = QString::fromStdString(drive_pair.first.as<std::string>());
        YAML::Node drive_node = drive_pair.second;
        
        // Get port_id for this drive (default 1 = real)
        int port_id = 1;
        if (port_ids_node && port_ids_node[drive_pair.first.as<std::string>()]) {
          port_id = port_ids_node[drive_pair.first.as<std::string>()].as<int>();
        }
        
        if (drive_node["joints"]) {
          for (const auto& joint_pair : drive_node["joints"]) {
            QString joint_name = QString::fromStdString(joint_pair.first.as<std::string>());
            YAML::Node joint_node = joint_pair.second;
            
            int row = joints_table_->rowCount();
            joints_table_->insertRow(row);
            
            QTableWidgetItem* joint_item = new QTableWidgetItem(joint_name);
            joint_item->setFlags(joint_item->flags() & ~Qt::ItemIsEditable);
            joints_table_->setItem(row, 0, joint_item);
            
            QTableWidgetItem* drive_item = new QTableWidgetItem(drive_id);
            drive_item->setFlags(drive_item->flags() & ~Qt::ItemIsEditable);
            joints_table_->setItem(row, 1, drive_item);
            
            // Port ID (editable: 0=sim, 1=real)
            QSpinBox* port_spin = new QSpinBox();
            port_spin->setRange(0, 1);
            port_spin->setValue(port_id);
            port_spin->setToolTip("0 = Simulation, 1 = Real (SOEM)");
            port_spin->setStyleSheet("background: #1e1e1e; color: white;");
            joints_table_->setCellWidget(row, 2, port_spin);
            
            QSpinBox* gear_spin = new QSpinBox();
            gear_spin->setRange(-1000, 1000);
            gear_spin->setValue(joint_node["gear_ratio"] ? joint_node["gear_ratio"].as<int>() : 1);
            gear_spin->setStyleSheet("background: #1e1e1e; color: white;");
            joints_table_->setCellWidget(row, 3, gear_spin);
            
            QSpinBox* enc_spin = new QSpinBox();
            enc_spin->setRange(0, 100000);
            enc_spin->setValue(joint_node["encoder_res"] ? joint_node["encoder_res"].as<int>() : 10000);
            enc_spin->setStyleSheet("background: #1e1e1e; color: white;");
            joints_table_->setCellWidget(row, 4, enc_spin);
            
            QSpinBox* offset_spin = new QSpinBox();
            offset_spin->setRange(-100000, 100000);
            offset_spin->setValue(joint_node["encoder_offset"] ? joint_node["encoder_offset"].as<int>() : 0);
            offset_spin->setStyleSheet("background: #1e1e1e; color: white;");
            joints_table_->setCellWidget(row, 5, offset_spin);
            
            QCheckBox* log_check = new QCheckBox();
            log_check->setChecked(joint_node["log_joint"] ? joint_node["log_joint"].as<bool>() : false);
            QWidget* check_container = new QWidget();
            QHBoxLayout* check_layout = new QHBoxLayout(check_container);
            check_layout->addWidget(log_check);
            check_layout->setAlignment(Qt::AlignCenter);
            check_layout->setContentsMargins(0, 0, 0, 0);
            joints_table_->setCellWidget(row, 6, check_container);
          }
        }
      }
    }
    
    // Sort table by joint name (column 0)
    joints_table_->sortItems(0, Qt::AscendingOrder);
    
    appendLog("✓ Loaded from ar_drive.yaml", "#4ec9b0");
    
  } catch (const std::exception& e) {
    appendLog(QString("✗ Load error: %1").arg(e.what()), "#f14c4c");
  }
}

void ParamEditorWindow::appendLog(const QString& msg, const QString& color)
{
  QString timestamp = QTime::currentTime().toString("HH:mm:ss");
  QString clean_msg = msg;
  clean_msg.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;");
  
  QString html = QString("<span style='color: #555;'>[%1]</span> <span style='color: %2;'>%3</span>")
                   .arg(timestamp).arg(color).arg(clean_msg);
                   
  // System messages go directly to bringup log (low volume, safe to be immediate)
  bringup_log_->append(html);
  bringup_log_->verticalScrollBar()->setValue(bringup_log_->verticalScrollBar()->maximum());
}

bool ParamEditorWindow::isProcessRunning(const QString& pattern) const
{
  QProcess proc;
  proc.start("pgrep", QStringList() << "-f" << pattern);
  if (!proc.waitForFinished(500)) {
    return false;
  }

  const bool has_output = !proc.readAllStandardOutput().isEmpty();
  return proc.exitStatus() == QProcess::NormalExit && proc.exitCode() == 0 && has_output;
}

void ParamEditorWindow::killAllRosProcesses()
{
  appendLog("🧹 Cleaning up existing ROS processes...", "#ffcc00");
  
  // Kill all ROS-related processes to ensure clean state
  // NOTE: Do NOT kill param_editor or clear shared memory - UI needs them!
  QProcess killer;
  killer.start("bash", QStringList() << "-c" << 
    "pkill -9 -f 'ros2_control_node|move_group|robot_state_publisher|"
    "static_transform_publisher|controller_manager|spawner|ar_control_server|rviz2' 2>/dev/null; "
    "sleep 0.3");
  killer.waitForFinished(3000);
  
  appendLog("✓ Cleanup complete", "#4ec9b0");
}

bool ParamEditorWindow::validatePortIdsForRealRobot(QString& error_msg) const
{
  // Collect port_ids in drive order (drive_1, drive_2, etc.)
  std::map<int, int> drive_port_map;  // drive_number -> port_id
  
  for (int row = 0; row < joints_table_->rowCount(); ++row) {
    QString drive_id = joints_table_->item(row, 1)->text();  // e.g., "drive_1"
    QSpinBox* port_spin = qobject_cast<QSpinBox*>(joints_table_->cellWidget(row, 2));
    
    // Extract drive number from drive_id (e.g., "drive_1" -> 1)
    int drive_num = drive_id.mid(6).toInt();  // Skip "drive_" prefix
    int port_id = port_spin ? port_spin->value() : 1;
    
    drive_port_map[drive_num] = port_id;
  }
  
  // Check for gaps: once we see a 0, all subsequent drives must also be 0
  // Valid: 1,1,1,1,1 or 1,1,1,0,0 or 0,0,0,0,0
  // Invalid: 1,1,0,1,1 (gap in middle)
  bool seen_zero = false;
  for (const auto& [drive_num, port_id] : drive_port_map) {
    if (port_id == 0) {
      seen_zero = true;
    } else if (seen_zero && port_id == 1) {
      // Found a 1 after seeing a 0 - invalid gap!
      error_msg = QString("❌ Invalid port_ids: drive_%1 is real (1) but comes after a simulated drive (0).\n\n"
                         "EtherCAT requires continuous connection.\n"
                         "Real drives must be consecutive without gaps.").arg(drive_num);
      return false;
    }
  }
  
  return true;
}

bool ParamEditorWindow::validateDualAxisDrivers(QString& error_msg) const
{
  // Only 2CL3 drivers can be dual-axis (have 2 joints)
  // Count joints per drive
  std::map<QString, int> joints_per_drive;
  
  for (int row = 0; row < joints_table_->rowCount(); ++row) {
    QString drive_id = joints_table_->item(row, 1)->text();
    joints_per_drive[drive_id]++;
  }
  
  // Check: if a drive has more than 1 joint, it must be a 2CL3 driver
  // We need to reload the original driver_name from yaml to check this
  try {
    std::string config_dir = ament_index_cpp::get_package_share_directory("ar_control");
    std::string yaml_path = config_dir + "/config/ar_drive.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path);
    
    if (config["drives"]) {
      for (const auto& [drive_id, joint_count] : joints_per_drive) {
        if (joint_count > 1) {
          // This drive has multiple joints - check if it's 2CL3
          std::string drive_key = drive_id.toStdString();
          if (config["drives"][drive_key] && config["drives"][drive_key]["driver_name"]) {
            std::string driver_name = config["drives"][drive_key]["driver_name"].as<std::string>();
            // Check if it's a 2CL3 variant (2CL3-EC403T, 2CL3-EC507T, etc.)
            if (driver_name.find("2CL3") == std::string::npos) {
              error_msg = QString("❌ Invalid dual-axis: %1 has %2 joints but uses %3 driver.\n\n"
                                 "Only 2CL3 series drivers support dual-axis (2 joints).")
                          .arg(drive_id).arg(joint_count).arg(QString::fromStdString(driver_name));
              return false;
            }
          }
        }
      }
    }
  } catch (const std::exception& e) {
    // If we can't check, allow it (warning will be shown elsewhere)
    error_msg = QString("⚠️ Could not validate dual-axis config: %1").arg(e.what());
    return true;  // Allow to proceed, just warn
  }
  
  return true;
}

void ParamEditorWindow::bufferLog(QTextEdit* target, const QString& msg)
{
  // FAST PATH: Just store raw string. No parsing here.
  // This runs on main thread but is very cheap.
  log_buffers_[target].append(msg);
}

void ParamEditorWindow::onFlushLogs()
{
  static const QRegularExpression ansi_regex("\\x1b\\[([0-9;]+)m|\\[([0-9]+)m");
  static const QRegularExpression remove_regex("\\x1b\\[[HJK]|\\[H\\[2J\\[3J");
  static const QMap<QString, QString> ansi_colors = {
    {"30", "#000000"}, {"31", "#f14c4c"}, {"32", "#4ec9b0"}, {"33", "#ffcc00"},
    {"34", "#569cd6"}, {"35", "#c586c0"}, {"36", "#4ec9b0"}, {"37", "#d4d4d4"},
    {"90", "#808080"}, {"91", "#f14c4c"}, {"92", "#4ec9b0"}, {"93", "#ffcc00"},
    {"94", "#569cd6"}, {"0", "#d4d4d4"}
  };

  QString timestamp = QTime::currentTime().toString("HH:mm:ss");
  
  for (auto it = log_buffers_.begin(); it != log_buffers_.end(); ++it) {
    QTextEdit* target = it.key();
    QStringList& buffer = it.value();
    
    if (buffer.isEmpty()) continue;
    
    // FLOOD CONTROL: If buffer is huge, process only the last 200 lines
    bool flooded = false;
    int dropped_count = 0;
    if (buffer.size() > 200) {
       dropped_count = buffer.size() - 200;
       // Erase from beginning
       buffer.erase(buffer.begin(), buffer.begin() + dropped_count);
       flooded = true;
    }
    
    // Process buffer into HTML
    QString bulk_html;
    if (flooded) {
       bulk_html += QString("<span style='color: #f14c4c;'>... [SKIPPED %1 LINES DUE TO FLOOD] ...</span><br>").arg(dropped_count);
    }
    
    QString current_color = "#d4d4d4"; // Reset color per flush, or track it? 
    // Tracking across flushes is hard without state. Resetting is acceptable.
    
    for (const QString& line : buffer) {
       QString clean_msg = line;
       clean_msg.remove(remove_regex);
       if (clean_msg.trimmed().isEmpty()) continue; // Skip empty
       
       QString html_line_msg;
       QRegularExpressionMatchIterator iter = ansi_regex.globalMatch(clean_msg);
       int lastPos = 0;
       
       // ANSI Parsing Loop
       while (iter.hasNext()) {
         QRegularExpressionMatch match = iter.next();
         QString text_part = clean_msg.mid(lastPos, match.capturedStart() - lastPos);
         if (!text_part.isEmpty()) {
           html_line_msg += QString("<span style='color: %1;'>%2</span>").arg(current_color).arg(text_part.toHtmlEscaped());
         }
         
         QString code = match.captured(1).isEmpty() ? match.captured(2) : match.captured(1);
         for (const QString& c : code.split(";")) {
           if (ansi_colors.contains(c)) current_color = ansi_colors[c];
         }
         lastPos = match.capturedEnd();
       }
       
       QString remaining = clean_msg.mid(lastPos);
       if (!remaining.isEmpty()) {
         html_line_msg += QString("<span style='color: %1;'>%2</span>").arg(current_color).arg(remaining.toHtmlEscaped());
       }
       
       if (!html_line_msg.isEmpty()) {
          bulk_html += QString("<span style='color: #555;'>[%1]</span> %2<br>").arg(timestamp).arg(html_line_msg);
       }
    }
    
    buffer.clear();
    
    // Append to UI (Efficient Op)
    if (!bulk_html.isEmpty()) {
       target->append(bulk_html);
    }
    
    target->verticalScrollBar()->setValue(target->verticalScrollBar()->maximum());
  }
}

void ParamEditorWindow::onStopProcess()
{
  QWidget* current = log_tabs_->currentWidget();
  QProcess* proc = nullptr;
  QString name;
  
  if (current == bringup_log_) { proc = bringup_process_; name = "Bringup"; }
  else if (current == moveit_log_) { proc = moveit_process_; name = "MoveIt"; }
  else if (current == rviz_log_) { proc = rviz_process_; name = "RViz"; }
  
  auto clearProc = [](QProcess*& p) {
    if (p) {
      p->disconnect();
      p->deleteLater();
      p = nullptr;
    }
  };

  if (proc && proc->state() == QProcess::Running) {
    appendLog(QString("🛑 Stopping %1 process group...").arg(name), "#f14c4c");
    
    qint64 pid = proc->processId();
    if (pid > 0) {
      // Kill entire process group with SIGTERM first
      ::kill(-pid, SIGTERM);
    }
    
    QTimer::singleShot(2000, this, [proc, name, this, current, pid]() {
        if (proc && proc->state() == QProcess::Running) {
             // Force kill the entire process group
             if (pid > 0) {
               ::kill(-pid, SIGKILL);
             }
             proc->kill();
             QString msg = QString("<span style='color: #f14c4c;'>☠️ Force killed %1 process group</span>").arg(name);
             if (current) log_buffers_[(QTextEdit*)current].append(msg);
        }
    });
    connect(proc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this, current, clearProc](int, QProcess::ExitStatus) {
      if (current == bringup_log_) {
        clearProc(bringup_process_);
        bringup_complete_ = false;
      } else if (current == moveit_log_) {
        clearProc(moveit_process_);
      } else if (current == rviz_log_) {
        clearProc(rviz_process_);
      }
    });
    // If bringup is being stopped, require new bringup before MoveIt/RViz
    if (current == bringup_log_) {
      bringup_complete_ = false;
      status_label_->setText("⚪ Ready");
      status_label_->setStyleSheet("color: #808080; font-weight: bold; font-size: 12px;");
    }
  } else if (proc) {
    if (proc == bringup_process_) {
      clearProc(bringup_process_);
      bringup_complete_ = false;
    } else if (proc == moveit_process_) {
      clearProc(moveit_process_);
    } else if (proc == rviz_process_) {
      clearProc(rviz_process_);
    }
    appendLog(QString("ℹ️ %1 process already stopped").arg(name), "#ffcc00");
  } else {
    appendLog("⚠️ No running process found for active tab", "#ffcc00");
  }
}

QString ParamEditorWindow::generateYamlFromUi()
{
  QString yaml;
  QTextStream out(&yaml);
  
  out << "### ar_drive.yaml - Generated from UI\n\n";
  out << "port_info:\n";
  out << "  port_name: " << port_name_edit_->text() << "\n\n";
  out << "driver_info:\n";
  out << "  driver_mode: " << driver_mode_spin_->value() << "\n\n";
  out << "use_torque_offset: " << (use_torque_offset_check_->isChecked() ? "true" : "false") << "\n\n";
  
  // Collect port_ids and joint params per drive
  // port_id, gear, enc, offset, log
  std::map<QString, std::tuple<int, std::map<QString, std::tuple<int, int, int, bool>>>> drives;
  
  for (int row = 0; row < joints_table_->rowCount(); ++row) {
    QString joint_name = joints_table_->item(row, 0)->text();
    QString drive_id = joints_table_->item(row, 1)->text();
    
    QSpinBox* port_spin = qobject_cast<QSpinBox*>(joints_table_->cellWidget(row, 2));
    QSpinBox* gear_spin = qobject_cast<QSpinBox*>(joints_table_->cellWidget(row, 3));
    QSpinBox* enc_spin = qobject_cast<QSpinBox*>(joints_table_->cellWidget(row, 4));
    QSpinBox* offset_spin = qobject_cast<QSpinBox*>(joints_table_->cellWidget(row, 5));
    QWidget* check_container = joints_table_->cellWidget(row, 6);
    QCheckBox* log_check = check_container ? check_container->findChild<QCheckBox*>() : nullptr;
    
    int port = port_spin ? port_spin->value() : 1;
    int gear = gear_spin ? gear_spin->value() : 1;
    int enc = enc_spin ? enc_spin->value() : 10000;
    int offset = offset_spin ? offset_spin->value() : 0;
    bool log = log_check ? log_check->isChecked() : false;
    
    // Initialize drive with port_id if first joint
    if (drives.find(drive_id) == drives.end()) {
      drives[drive_id] = std::make_tuple(port, std::map<QString, std::tuple<int, int, int, bool>>());
    }
    std::get<1>(drives[drive_id])[joint_name] = std::make_tuple(gear, enc, offset, log);
  }
  
  // Output port_ids
  out << "port_ids:\n";
  for (const auto& drive_pair : drives) {
    int port_id = std::get<0>(drive_pair.second);
    out << "  " << drive_pair.first << ": " << port_id << "\n";
  }
  out << "\n";
  
  out << "drives:\n";
  for (const auto& drive_pair : drives) {
    QString drive_id = drive_pair.first;
    const auto& joints_map = std::get<1>(drive_pair.second);
    out << "  " << drive_id << ":\n";
    out << "    is_dual_axis: " << (joints_map.size() > 1 ? "true" : "false") << "\n";
    out << "    driver_name: CS3E-D503B\n";
    out << "    joints:\n";
    
    for (const auto& joint_pair : joints_map) {
      QString joint_name = joint_pair.first;
      auto& params = joint_pair.second;
      
      out << "      " << joint_name << ":\n";
      out << "        gear_ratio: " << std::get<0>(params) << "\n";
      out << "        encoder_res: " << std::get<1>(params) << "\n";
      out << "        encoder_offset: " << std::get<2>(params) << "\n";
      out << "        log_joint: " << (std::get<3>(params) ? "true" : "false") << "\n";
    }
  }
  
  return yaml;
}

void ParamEditorWindow::saveYamlToFile(const QString& yaml_content)
{
  // Only save to temp file for debugging - DO NOT modify workspace files
  QFile tmpFile("/tmp/ar_drive_edited.yaml");
  if (tmpFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QTextStream out(&tmpFile);
    out << yaml_content;
    tmpFile.close();
  }
  // Parameters are sent via ROS topic only - no workspace files are modified
}

void ParamEditorWindow::onReloadFromFile()
{
  appendLog("🔄 Reloading from file...", "#569cd6");
  loadYamlToUi();
  status_label_->setText("✓ Reloaded");
  status_label_->setStyleSheet("color: #4ec9b0; font-weight: bold; font-size: 12px;");
}

void ParamEditorWindow::onLoadParams()
{
  appendLog("📊 Publishing parameters to server...", "#569cd6");
  
  status_label_->setText("⏳ Loading...");
  status_label_->setStyleSheet("color: #FF9800; font-weight: bold; font-size: 12px;");
  
  QString yaml = generateYamlFromUi();
  saveYamlToFile(yaml);  // Only saves to /tmp for debugging
  
  emit publishRequested(yaml);
  
  appendLog("✓ Parameters published to /ar_params/loaded", "#4ec9b0");
  RCLCPP_INFO(node_->get_logger(), "Parameters publication requested");
  
  status_label_->setText("✓ Params Published");
  status_label_->setStyleSheet("color: #4ec9b0; font-weight: bold; font-size: 12px;");
}

void ParamEditorWindow::onRunRobot()
{
  // Validate port_ids for real robot - must be continuous (no gaps like 1,1,0,1,1)
  QString error_msg;
  if (!validatePortIdsForRealRobot(error_msg)) {
    QMessageBox::critical(this, "Configuration Error", error_msg);
    appendLog(error_msg.split('\n').first(), "#f14c4c");
    return;
  }
  
  // Validate dual-axis drivers (only 2CL3 can have 2 joints)
  if (!validateDualAxisDrivers(error_msg)) {
    QMessageBox::critical(this, "Configuration Error", error_msg);
    appendLog(error_msg.split('\n').first(), "#f14c4c");
    return;
  }
  
  // Kill any existing ROS processes first
  killAllRosProcesses();
  
  // Generate YAML now but publish AFTER bringup starts (so hardware interface can receive it)
  QString yaml = generateYamlFromUi();
  saveYamlToFile(yaml);  // Save to /tmp for debugging
  
  appendLog("🤖 Starting robot bringup...", "#FF5722");
  
  status_label_->setText("⏳ Starting Robot...");
  status_label_->setStyleSheet("color: #FF5722; font-weight: bold; font-size: 12px;");
  bringup_complete_ = false;
  
  load_params_btn_->setEnabled(false);
  run_robot_btn_->setEnabled(false);
  run_simulation_btn_->setEnabled(false);
  
  // Terminate old process if exists
  if (bringup_process_) {
    bringup_process_->disconnect();
    if (bringup_process_->state() == QProcess::Running) {
      bringup_process_->terminate();
      bringup_process_->waitForFinished(1000);
    }
    delete bringup_process_;
    bringup_process_ = nullptr;
  }
  
  // Switch to bringup log tab
  log_tabs_->setCurrentWidget(bringup_log_);
  
  bringup_process_ = new QProcess(this);
  
  // Capture stdout - batch entire chunks to reduce overhead
  connect(bringup_process_, &QProcess::readyReadStandardOutput, this, [this]() {
    if (!bringup_process_) return;
    QByteArray data = bringup_process_->readAllStandardOutput();
    if (!data.isEmpty()) {
      bufferLog(bringup_log_, QString::fromUtf8(data).trimmed());
    }
  });
  
  // Capture stderr
  connect(bringup_process_, &QProcess::readyReadStandardError, this, [this]() {
    if (!bringup_process_) return;
    QByteArray data = bringup_process_->readAllStandardError();
    if (!data.isEmpty()) {
      bufferLog(bringup_log_, QString::fromUtf8(data).trimmed());
    }
  });
  
  connect(bringup_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, [this](int exitCode, QProcess::ExitStatus status) {
    if (status == QProcess::NormalExit && exitCode == 0) {
      appendLog("✓ Robot ready", "#4ec9b0");
      status_label_->setText("✓ Robot Ready");
      status_label_->setStyleSheet("color: #4ec9b0; font-weight: bold; font-size: 12px;");
      bringup_complete_ = true;
    } else {
      appendLog(QString("✗ Bringup failed (code: %1)").arg(exitCode), "#f14c4c");
      status_label_->setText("✗ Bringup Failed");
      status_label_->setStyleSheet("color: #f14c4c; font-weight: bold; font-size: 12px;");
      bringup_complete_ = false;
    }
    bringup_process_ = nullptr;
    load_params_btn_->setEnabled(true);
    run_robot_btn_->setEnabled(true);
    run_simulation_btn_->setEnabled(true);
  });
  
  // Use setsid to create a new process group so we can kill all children
  QString cmd = QString("ros2 launch ar_control ar_bringup.launch.py desc:=%1").arg(robot_desc_edit_->text());
  bringup_process_->start("setsid", QStringList() 
    << "bash" << "-c" << cmd);
  appendLog("📡 " + cmd, "#9cdcfe");
  
  // Publish parameters multiple times during the hardware interface's 3s wait window
  // This ensures discovery and delivery even with DDS timing variations
  appendLog("📊 Publishing parameters to server...", "#569cd6");
  
  // Publish immediately (in case discovery already happened)
  emit publishRequested(yaml);
  
  // Publish again at 0.5s, 1.0s, 1.5s, 2.0s to cover the 3s wait window
  for (int delay_ms : {500, 1000, 1500, 2000}) {
    QTimer::singleShot(delay_ms, this, [this, yaml]() {
      emit publishRequested(yaml);
    });
  }
  
  QTimer::singleShot(2100, this, [this]() {
    appendLog("✓ Parameters published to /ar_params/loaded", "#4ec9b0");
    RCLCPP_INFO(node_->get_logger(), "Parameters publication completed");
  });
  
  // Set bringup complete after a short delay (ros2 launch doesn't exit)
  QTimer::singleShot(5000, this, [this]() {
    if (bringup_process_ && bringup_process_->state() == QProcess::Running) {
      bringup_complete_ = true;
      appendLog("✓ Bringup running - MoveIt/RViz now available", "#4ec9b0");
    }
  });
}

void ParamEditorWindow::onRunSimulation()
{
  // Kill any existing ROS processes first
  killAllRosProcesses();
  
  // Generate YAML now but publish AFTER bringup starts (so hardware interface can receive it)
  QString yaml = generateYamlFromUi();
  saveYamlToFile(yaml);  // Save to /tmp for debugging
  
  appendLog("🎮 Starting simulation...", "#9C27B0");
  
  status_label_->setText("⏳ Starting Simulation...");
  status_label_->setStyleSheet("color: #9C27B0; font-weight: bold; font-size: 12px;");
  bringup_complete_ = false;
  
  load_params_btn_->setEnabled(false);
  run_robot_btn_->setEnabled(false);
  run_simulation_btn_->setEnabled(false);
  
  // Terminate old process if exists
  if (bringup_process_) {
    bringup_process_->disconnect();
    if (bringup_process_->state() == QProcess::Running) {
      bringup_process_->terminate();
      bringup_process_->waitForFinished(1000);
    }
    delete bringup_process_;
    bringup_process_ = nullptr;
  }
  
  // Switch to bringup log tab
  log_tabs_->setCurrentWidget(bringup_log_);
  
  bringup_process_ = new QProcess(this);
  
  // Capture stdout - batch entire chunks to reduce overhead
  connect(bringup_process_, &QProcess::readyReadStandardOutput, this, [this]() {
    if (!bringup_process_) return;
    QByteArray data = bringup_process_->readAllStandardOutput();
    if (!data.isEmpty()) {
      bufferLog(bringup_log_, QString::fromUtf8(data).trimmed());
    }
  });
  
  // Capture stderr
  connect(bringup_process_, &QProcess::readyReadStandardError, this, [this]() {
    if (!bringup_process_) return;
    QByteArray data = bringup_process_->readAllStandardError();
    if (!data.isEmpty()) {
      bufferLog(bringup_log_, QString::fromUtf8(data).trimmed());
    }
  });
  
  connect(bringup_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, [this](int exitCode, QProcess::ExitStatus status) {
    if (status == QProcess::NormalExit && exitCode == 0) {
      appendLog("✓ Simulation ready", "#4ec9b0");
      status_label_->setText("✓ Simulation Ready");
      status_label_->setStyleSheet("color: #4ec9b0; font-weight: bold; font-size: 12px;");
      bringup_complete_ = true;
    } else {
      appendLog(QString("✗ Simulation failed (code: %1)").arg(exitCode), "#f14c4c");
      status_label_->setText("✗ Simulation Failed");
      status_label_->setStyleSheet("color: #f14c4c; font-weight: bold; font-size: 12px;");
      bringup_complete_ = false;
    }
    bringup_process_ = nullptr;
    load_params_btn_->setEnabled(true);
    run_robot_btn_->setEnabled(true);
    run_simulation_btn_->setEnabled(true);
  });
  
  // Use setsid to create a new process group so we can kill all children
  QString cmd = QString("ros2 launch ar_control ar_bringup.launch.py desc:=%1 sim:=true").arg(robot_desc_edit_->text());
  bringup_process_->start("setsid", QStringList() 
    << "bash" << "-c" << cmd);
  appendLog("📡 " + cmd, "#9cdcfe");
  
  // Publish parameters multiple times during the hardware interface's 3s wait window
  // This ensures discovery and delivery even with DDS timing variations
  appendLog("📊 Publishing parameters to server...", "#569cd6");
  
  // Publish immediately (in case discovery already happened)
  emit publishRequested(yaml);
  
  // Publish again at 0.5s, 1.0s, 1.5s, 2.0s to cover the 3s wait window
  for (int delay_ms : {500, 1000, 1500, 2000}) {
    QTimer::singleShot(delay_ms, this, [this, yaml]() {
      emit publishRequested(yaml);
    });
  }
  
  QTimer::singleShot(2100, this, [this]() {
    appendLog("✓ Parameters published to /ar_params/loaded", "#4ec9b0");
    RCLCPP_INFO(node_->get_logger(), "Parameters publication completed");
  });
  
  // Set bringup complete after a short delay (ros2 launch doesn't exit)
  QTimer::singleShot(5000, this, [this]() {
    if (bringup_process_ && bringup_process_->state() == QProcess::Running) {
      bringup_complete_ = true;
      appendLog("✓ Simulation running - MoveIt/RViz now available", "#4ec9b0");
    }
  });
}

void ParamEditorWindow::onLaunchMoveIt()
{
  if (!bringup_complete_) {
    QMessageBox::warning(this, "Warning", 
      "⚠️ Please run Robot or Simulation bringup first!\n\n"
      "MoveIt requires the robot controllers to be running.");
    appendLog("⚠️ Cannot launch MoveIt - bringup not complete", "#ffcc00");
    return;
  }
  
  // Prevent double-launch if move_group is already running elsewhere
  if (isProcessRunning("move_group")) {
    QMessageBox::warning(this, "MoveIt already running",
      "Detected an existing move_group process.\n\n"
      "Stop the current MoveIt instance first or use the Stop button.");
    appendLog("⚠️ MoveIt already running (move_group detected) - skipping launch", "#ffcc00");
    return;
  }

  appendLog("🦾 Launching MoveIt...", "#2196F3");
  
  // Terminate old process if exists
  if (moveit_process_) {
    moveit_process_->disconnect();
    if (moveit_process_->state() == QProcess::Running) {
      appendLog("⚠️ MoveIt already running", "#ffcc00");
      return;
    }
    delete moveit_process_;
    moveit_process_ = nullptr;
  }
  
  // Switch to MoveIt log tab
  log_tabs_->setCurrentWidget(moveit_log_);
  
  moveit_process_ = new QProcess(this);
  
  connect(moveit_process_, &QProcess::readyReadStandardOutput, this, [this]() {
    if (!moveit_process_) return;
    QByteArray data = moveit_process_->readAllStandardOutput();
    if (!data.isEmpty()) {
      bufferLog(moveit_log_, QString::fromUtf8(data).trimmed());
    }
  });
  
  connect(moveit_process_, &QProcess::readyReadStandardError, this, [this]() {
    if (!moveit_process_) return;
    QByteArray data = moveit_process_->readAllStandardError();
    if (!data.isEmpty()) {
      bufferLog(moveit_log_, QString::fromUtf8(data).trimmed());
    }
  });
  
  // Use setsid to create a new process group so we can kill all children
  QString cmd = QString("ros2 launch ar_control ar_moveit.launch.py desc:=%1").arg(robot_desc_edit_->text());
  moveit_process_->start("setsid", QStringList() 
    << "bash" << "-c" << cmd);
  appendLog("📡 " + cmd, "#9cdcfe");
}

void ParamEditorWindow::onLaunchRViz()
{
  if (!bringup_complete_) {
    QMessageBox::warning(this, "Warning", 
      "⚠️ Please run Robot or Simulation bringup first!\n\n"
      "RViz requires the robot state to be published.");
    appendLog("⚠️ Cannot launch RViz - bringup not complete", "#ffcc00");
    return;
  }
  
  appendLog("👁️ Launching RViz...", "#00BCD4");
  
  // Terminate old process if exists
  if (rviz_process_) {
    rviz_process_->disconnect();
    if (rviz_process_->state() == QProcess::Running) {
      appendLog("⚠️ RViz already running", "#ffcc00");
      return;
    }
    delete rviz_process_;
    rviz_process_ = nullptr;
  }
  
  // Switch to RViz log tab
  log_tabs_->setCurrentWidget(rviz_log_);
  
  rviz_process_ = new QProcess(this);
  
  connect(rviz_process_, &QProcess::readyReadStandardOutput, this, [this]() {
    if (!rviz_process_) return;
    QByteArray data = rviz_process_->readAllStandardOutput();
    if (!data.isEmpty()) {
      bufferLog(rviz_log_, QString::fromUtf8(data).trimmed());
    }
  });
  
  connect(rviz_process_, &QProcess::readyReadStandardError, this, [this]() {
    if (!rviz_process_) return;
    QByteArray data = rviz_process_->readAllStandardError();
    if (!data.isEmpty()) {
      bufferLog(rviz_log_, QString::fromUtf8(data).trimmed());
    }
  });
  
  // Use setsid to create a new process group so we can kill all children
  QString cmd = QString("ros2 launch ar_control ar_rviz.launch.py desc:=%1").arg(robot_desc_edit_->text());
  rviz_process_->start("setsid", QStringList() 
    << "bash" << "-c" << cmd);
  appendLog("📡 " + cmd, "#9cdcfe");
}

}  // namespace ar_ui
