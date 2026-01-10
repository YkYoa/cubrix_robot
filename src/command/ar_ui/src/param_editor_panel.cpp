#include "ar_ui/param_editor_panel.h"

#include <QFile>
#include <QTextStream>
#include <QScrollBar>
#include <QTime>
#include <QHeaderView>
#include <QFontDatabase>
#include <QMessageBox>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rviz_common/display_context.hpp>
#include <yaml-cpp/yaml.h>

namespace ar_ui
{

ParamEditorPanel::ParamEditorPanel(QWidget* parent)
  : rviz_common::Panel(parent)
  , bringup_process_(nullptr)
{
  setupUi();
}

ParamEditorPanel::~ParamEditorPanel()
{
  if (bringup_process_ && bringup_process_->state() == QProcess::Running) {
    bringup_process_->terminate();
    bringup_process_->waitForFinished(2000);
  }
}

void ParamEditorPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  
  // Publisher for loaded parameters
  param_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/ar_params/loaded", rclcpp::QoS(10).transient_local());
  
  RCLCPP_INFO(node_->get_logger(), "ParamEditorPanel initialized");
  
  // Load YAML into UI
  loadYamlToUi();
  appendLog("🤖 Panel Ready", "#569cd6");
}

void ParamEditorPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void ParamEditorPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
}

void ParamEditorPanel::setupUi()
{
  main_layout_ = new QVBoxLayout(this);
  main_layout_->setContentsMargins(6, 6, 6, 6);
  main_layout_->setSpacing(6);
  
  // Title
  QLabel* title = new QLabel("🔧 Hardware Parameters", this);
  title->setStyleSheet("font-weight: bold; font-size: 13px;");
  title->setAlignment(Qt::AlignCenter);
  main_layout_->addWidget(title);
  
  // Tab widget
  tab_widget_ = new QTabWidget(this);
  tab_widget_->setStyleSheet(
    "QTabWidget::pane { border: 1px solid #3c3c3c; background: #2d2d2d; }"
    "QTabBar::tab { background: #1e1e1e; color: #ccc; padding: 6px 12px; }"
    "QTabBar::tab:selected { background: #0e639c; color: white; }");
  
  // ============ Settings Tab ============
  QWidget* settings_tab = new QWidget();
  QVBoxLayout* settings_layout = new QVBoxLayout(settings_tab);
  settings_layout->setSpacing(8);
  
  // Port settings
  QGroupBox* port_group = new QGroupBox("Port Settings", settings_tab);
  QGridLayout* port_layout = new QGridLayout(port_group);
  
  port_layout->addWidget(new QLabel("Port Name:"), 0, 0);
  port_name_edit_ = new QLineEdit("enp2s0", settings_tab);
  port_name_edit_->setStyleSheet("background: #1e1e1e; color: white; padding: 4px;");
  port_layout->addWidget(port_name_edit_, 0, 1);
  
  port_layout->addWidget(new QLabel("Driver Mode:"), 1, 0);
  driver_mode_spin_ = new QSpinBox(settings_tab);
  driver_mode_spin_->setRange(0, 1);
  driver_mode_spin_->setValue(1);
  driver_mode_spin_->setToolTip("0 = Profile Position, 1 = Cyclic Position");
  driver_mode_spin_->setStyleSheet("background: #1e1e1e; color: white; padding: 4px;");
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
  joints_table_->setColumnCount(6);
  joints_table_->setHorizontalHeaderLabels({"Joint", "Drive", "Gear Ratio", "Encoder Res", "Offset", "Log"});
  joints_table_->horizontalHeader()->setStretchLastSection(true);
  joints_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  joints_table_->setStyleSheet(
    "QTableWidget { background: #1e1e1e; color: white; gridline-color: #3c3c3c; }"
    "QHeaderView::section { background: #2d2d2d; color: white; padding: 4px; border: 1px solid #3c3c3c; }"
    "QTableWidget::item { padding: 2px; }");
  joints_table_->setAlternatingRowColors(true);
  
  joints_layout->addWidget(joints_table_);
  
  tab_widget_->addTab(joints_tab, "🦾 Joints");
  
  main_layout_->addWidget(tab_widget_, 1);
  
  // ============ Action Buttons ============
  QHBoxLayout* btn_row1 = new QHBoxLayout();
  
  reload_btn_ = new QPushButton("🔄 Reload", this);
  reload_btn_->setStyleSheet(
    "QPushButton { background: #555; color: white; padding: 8px; border-radius: 4px; }"
    "QPushButton:hover { background: #666; }");
  reload_btn_->setToolTip("Reload parameters from ar_drive.yaml");
  btn_row1->addWidget(reload_btn_);
  
  main_layout_->addLayout(btn_row1);
  
  QHBoxLayout* btn_row2 = new QHBoxLayout();
  
  load_params_btn_ = new QPushButton("📊 Load Parameters", this);
  load_params_btn_->setStyleSheet(
    "QPushButton { background: #4CAF50; color: white; font-weight: bold; padding: 12px; border-radius: 5px; }"
    "QPushButton:hover { background: #66BB6A; }"
    "QPushButton:disabled { background: #555; }");
  load_params_btn_->setToolTip("Apply edited parameters to system");
  btn_row2->addWidget(load_params_btn_);
  
  main_layout_->addLayout(btn_row2);
  
  // Run buttons row
  QHBoxLayout* btn_row3 = new QHBoxLayout();
  
  run_robot_btn_ = new QPushButton("🤖 Run Robot", this);
  run_robot_btn_->setStyleSheet(
    "QPushButton { background: #FF5722; color: white; font-weight: bold; padding: 12px; border-radius: 5px; }"
    "QPushButton:hover { background: #FF7043; }"
    "QPushButton:disabled { background: #555; }");
  run_robot_btn_->setToolTip("Start hardware bringup (real robot)");
  btn_row3->addWidget(run_robot_btn_);
  
  run_simulation_btn_ = new QPushButton("🎮 Run Simulation", this);
  run_simulation_btn_->setStyleSheet(
    "QPushButton { background: #9C27B0; color: white; font-weight: bold; padding: 12px; border-radius: 5px; }"
    "QPushButton:hover { background: #AB47BC; }"
    "QPushButton:disabled { background: #555; }");
  run_simulation_btn_->setToolTip("Start simulation bringup");
  btn_row3->addWidget(run_simulation_btn_);
  
  main_layout_->addLayout(btn_row3);
  
  // Status
  status_label_ = new QLabel("⚪ Ready", this);
  status_label_->setStyleSheet("color: #808080; font-weight: bold; font-size: 11px;");
  status_label_->setAlignment(Qt::AlignCenter);
  main_layout_->addWidget(status_label_);
  
  // Log
  log_text_ = new QTextEdit(this);
  log_text_->setReadOnly(true);
  QFont log_font = QFontDatabase::systemFont(QFontDatabase::FixedFont);
  log_font.setPointSize(8);
  log_text_->setFont(log_font);
  log_text_->setStyleSheet(
    "background-color: #0d1117; color: #d4d4d4; border: 1px solid #3c3c3c; "
    "border-radius: 3px; padding: 4px;");
  log_text_->setMaximumHeight(100);
  main_layout_->addWidget(log_text_);
  
  // Connections
  connect(reload_btn_, &QPushButton::clicked, this, &ParamEditorPanel::onReloadFromFile);
  connect(load_params_btn_, &QPushButton::clicked, this, &ParamEditorPanel::onLoadParams);
  connect(run_robot_btn_, &QPushButton::clicked, this, &ParamEditorPanel::onRunRobot);
  connect(run_simulation_btn_, &QPushButton::clicked, this, &ParamEditorPanel::onRunSimulation);
}

void ParamEditorPanel::loadYamlToUi()
{
  try {
    std::string config_dir = ament_index_cpp::get_package_share_directory("ar_control");
    std::string yaml_path = config_dir + "/config/ar_drive.yaml";
    
    YAML::Node config = YAML::LoadFile(yaml_path);
    
    // Load port settings
    if (config["port_info"] && config["port_info"]["port_name"]) {
      port_name_edit_->setText(QString::fromStdString(config["port_info"]["port_name"].as<std::string>()));
    }
    if (config["driver_info"] && config["driver_info"]["driver_mode"]) {
      driver_mode_spin_->setValue(config["driver_info"]["driver_mode"].as<int>());
    }
    if (config["use_torque_offset"]) {
      use_torque_offset_check_->setChecked(config["use_torque_offset"].as<bool>());
    }
    
    // Load joints into table
    joints_table_->setRowCount(0);
    
    if (config["drives"]) {
      for (const auto& drive_pair : config["drives"]) {
        QString drive_id = QString::fromStdString(drive_pair.first.as<std::string>());
        YAML::Node drive_node = drive_pair.second;
        
        if (drive_node["joints"]) {
          for (const auto& joint_pair : drive_node["joints"]) {
            QString joint_name = QString::fromStdString(joint_pair.first.as<std::string>());
            YAML::Node joint_node = joint_pair.second;
            
            int row = joints_table_->rowCount();
            joints_table_->insertRow(row);
            
            // Joint name (read-only)
            QTableWidgetItem* joint_item = new QTableWidgetItem(joint_name);
            joint_item->setFlags(joint_item->flags() & ~Qt::ItemIsEditable);
            joints_table_->setItem(row, 0, joint_item);
            
            // Drive ID (read-only)
            QTableWidgetItem* drive_item = new QTableWidgetItem(drive_id);
            drive_item->setFlags(drive_item->flags() & ~Qt::ItemIsEditable);
            joints_table_->setItem(row, 1, drive_item);
            
            // Gear ratio (editable)
            QSpinBox* gear_spin = new QSpinBox();
            gear_spin->setRange(-1000, 1000);
            gear_spin->setValue(joint_node["gear_ratio"] ? joint_node["gear_ratio"].as<int>() : 1);
            gear_spin->setStyleSheet("background: #1e1e1e; color: white;");
            joints_table_->setCellWidget(row, 2, gear_spin);
            
            // Encoder res (editable)
            QSpinBox* enc_spin = new QSpinBox();
            enc_spin->setRange(0, 100000);
            enc_spin->setValue(joint_node["encoder_res"] ? joint_node["encoder_res"].as<int>() : 10000);
            enc_spin->setStyleSheet("background: #1e1e1e; color: white;");
            joints_table_->setCellWidget(row, 3, enc_spin);
            
            // Encoder offset (editable)
            QSpinBox* offset_spin = new QSpinBox();
            offset_spin->setRange(-100000, 100000);
            offset_spin->setValue(joint_node["encoder_offset"] ? joint_node["encoder_offset"].as<int>() : 0);
            offset_spin->setStyleSheet("background: #1e1e1e; color: white;");
            joints_table_->setCellWidget(row, 4, offset_spin);
            
            // Log joint (checkbox)
            QCheckBox* log_check = new QCheckBox();
            log_check->setChecked(joint_node["log_joint"] ? joint_node["log_joint"].as<bool>() : false);
            QWidget* check_container = new QWidget();
            QHBoxLayout* check_layout = new QHBoxLayout(check_container);
            check_layout->addWidget(log_check);
            check_layout->setAlignment(Qt::AlignCenter);
            check_layout->setContentsMargins(0, 0, 0, 0);
            joints_table_->setCellWidget(row, 5, check_container);
          }
        }
      }
    }
    
    appendLog("✓ Loaded from ar_drive.yaml", "#4ec9b0");
    
  } catch (const std::exception& e) {
    appendLog(QString("✗ Load error: %1").arg(e.what()), "#f14c4c");
  }
}

void ParamEditorPanel::appendLog(const QString& msg, const QString& color)
{
  QString timestamp = QTime::currentTime().toString("HH:mm:ss");
  QString html = QString("<span style='color: #666;'>[%1]</span> <span style='color: %2;'>%3</span>")
                   .arg(timestamp).arg(color).arg(msg);
  log_text_->append(html);
  log_text_->verticalScrollBar()->setValue(log_text_->verticalScrollBar()->maximum());
}

QString ParamEditorPanel::generateYamlFromUi()
{
  QString yaml;
  QTextStream out(&yaml);
  
  out << "### ar_drive.yaml - Generated from UI\n\n";
  
  // Port info
  out << "port_info:\n";
  out << "  port_name: " << port_name_edit_->text() << "\n\n";
  
  // Driver info
  out << "driver_info:\n";
  out << "  driver_mode: " << driver_mode_spin_->value() << "\n\n";
  
  // Torque offset
  out << "use_torque_offset: " << (use_torque_offset_check_->isChecked() ? "true" : "false") << "\n\n";
  
  // Collect drives and joints from table
  std::map<QString, std::map<QString, std::tuple<int, int, int, bool>>> drives;
  
  for (int row = 0; row < joints_table_->rowCount(); ++row) {
    QString joint_name = joints_table_->item(row, 0)->text();
    QString drive_id = joints_table_->item(row, 1)->text();
    
    QSpinBox* gear_spin = qobject_cast<QSpinBox*>(joints_table_->cellWidget(row, 2));
    QSpinBox* enc_spin = qobject_cast<QSpinBox*>(joints_table_->cellWidget(row, 3));
    QSpinBox* offset_spin = qobject_cast<QSpinBox*>(joints_table_->cellWidget(row, 4));
    QWidget* check_container = joints_table_->cellWidget(row, 5);
    QCheckBox* log_check = check_container ? check_container->findChild<QCheckBox*>() : nullptr;
    
    int gear = gear_spin ? gear_spin->value() : 1;
    int enc = enc_spin ? enc_spin->value() : 10000;
    int offset = offset_spin ? offset_spin->value() : 0;
    bool log = log_check ? log_check->isChecked() : false;
    
    drives[drive_id][joint_name] = std::make_tuple(gear, enc, offset, log);
  }
  
  // Port IDs (assuming PORT_SOEM = 1)
  out << "port_ids:\n";
  for (const auto& drive_pair : drives) {
    out << "  " << drive_pair.first << ": 1\n";
  }
  out << "\n";
  
  // Drives
  out << "drives:\n";
  for (const auto& drive_pair : drives) {
    QString drive_id = drive_pair.first;
    out << "  " << drive_id << ":\n";
    out << "    is_dual_axis: " << (drive_pair.second.size() > 1 ? "true" : "false") << "\n";
    out << "    driver_name: CS3E-D503B\n";
    out << "    joints:\n";
    
    for (const auto& joint_pair : drive_pair.second) {
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

void ParamEditorPanel::saveYamlToFile(const QString& yaml_content)
{
  // Save to temporary location (don't overwrite original)
  QFile file("/tmp/ar_drive_edited.yaml");
  if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QTextStream out(&file);
    out << yaml_content;
    file.close();
  }
}

void ParamEditorPanel::onReloadFromFile()
{
  appendLog("🔄 Reloading from file...", "#569cd6");
  loadYamlToUi();
  status_label_->setText("✓ Reloaded");
  status_label_->setStyleSheet("color: #4ec9b0; font-weight: bold;");
}

void ParamEditorPanel::onLoadParams()
{
  appendLog("📊 Loading parameters...", "#569cd6");
  
  status_label_->setText("⏳ Loading...");
  status_label_->setStyleSheet("color: #FF9800; font-weight: bold;");
  
  QString yaml = generateYamlFromUi();
  
  // Save to temp file
  saveYamlToFile(yaml);
  
  // Publish to topic
  if (param_pub_) {
    std_msgs::msg::String msg;
    msg.data = yaml.toStdString();
    param_pub_->publish(msg);
    
    appendLog("✓ Parameters loaded and published", "#4ec9b0");
    RCLCPP_INFO(node_->get_logger(), "Parameters published to /ar_params/loaded");
  }
  
  status_label_->setText("✓ Params Loaded");
  status_label_->setStyleSheet("color: #4ec9b0; font-weight: bold;");
}

void ParamEditorPanel::onRunRobot()
{
  // First load params
  onLoadParams();
  
  appendLog("🤖 Starting robot bringup...", "#FF5722");
  
  status_label_->setText("⏳ Starting Robot...");
  status_label_->setStyleSheet("color: #FF5722; font-weight: bold;");
  
  load_params_btn_->setEnabled(false);
  run_robot_btn_->setEnabled(false);
  run_simulation_btn_->setEnabled(false);
  
  if (!bringup_process_) {
    bringup_process_ = new QProcess(this);
    
    connect(bringup_process_, &QProcess::readyReadStandardOutput, this, [this]() {
      QString output = bringup_process_->readAllStandardOutput();
      for (const QString& line : output.split('\n', Qt::SkipEmptyParts)) {
        appendLog("[sys] " + line.trimmed(), "#9cdcfe");
      }
    });
    
    connect(bringup_process_, &QProcess::readyReadStandardError, this, [this]() {
      QString output = bringup_process_->readAllStandardError();
      for (const QString& line : output.split('\n', Qt::SkipEmptyParts)) {
        appendLog("[sys] " + line.trimmed(), "#dcdcaa");
      }
    });
    
    connect(bringup_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this](int exitCode, QProcess::ExitStatus status) {
      if (status == QProcess::NormalExit && exitCode == 0) {
        appendLog("✓ Robot ready", "#4ec9b0");
        status_label_->setText("✓ Robot Ready");
        status_label_->setStyleSheet("color: #4ec9b0; font-weight: bold;");
      } else {
        appendLog(QString("✗ Bringup failed (code: %1)").arg(exitCode), "#f14c4c");
        status_label_->setText("✗ Bringup Failed");
        status_label_->setStyleSheet("color: #f14c4c; font-weight: bold;");
      }
      load_params_btn_->setEnabled(true);
      run_robot_btn_->setEnabled(true);
      run_simulation_btn_->setEnabled(true);
    });
  }
  
  if (bringup_process_->state() != QProcess::Running) {
    // Launch robot bringup
    bringup_process_->start("ros2", QStringList() 
      << "launch" << "ar_control" << "ar_bringup.launch.py");
    
    appendLog("📡 Robot bringup started", "#9cdcfe");
  } else {
    appendLog("⚠️ Bringup already running", "#dcdcaa");
    load_params_btn_->setEnabled(true);
    run_robot_btn_->setEnabled(true);
    run_simulation_btn_->setEnabled(true);
  }
}

void ParamEditorPanel::onRunSimulation()
{
  // First load params
  onLoadParams();
  
  appendLog("🎮 Starting simulation...", "#9C27B0");
  
  status_label_->setText("⏳ Starting Simulation...");
  status_label_->setStyleSheet("color: #9C27B0; font-weight: bold;");
  
  load_params_btn_->setEnabled(false);
  run_robot_btn_->setEnabled(false);
  run_simulation_btn_->setEnabled(false);
  
  if (!bringup_process_) {
    bringup_process_ = new QProcess(this);
    
    connect(bringup_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this](int exitCode, QProcess::ExitStatus status) {
      if (status == QProcess::NormalExit && exitCode == 0) {
        appendLog("✓ Simulation ready", "#4ec9b0");
        status_label_->setText("✓ Simulation Ready");
        status_label_->setStyleSheet("color: #4ec9b0; font-weight: bold;");
      } else {
        appendLog(QString("✗ Simulation failed (code: %1)").arg(exitCode), "#f14c4c");
        status_label_->setText("✗ Simulation Failed");
        status_label_->setStyleSheet("color: #f14c4c; font-weight: bold;");
      }
      load_params_btn_->setEnabled(true);
      run_robot_btn_->setEnabled(true);
      run_simulation_btn_->setEnabled(true);
    });
  }
  
  if (bringup_process_->state() != QProcess::Running) {
    // Launch simulation bringup with simulation flag
    bringup_process_->start("ros2", QStringList() 
      << "launch" << "ar_control" << "ar_bringup.launch.py" << "simulation:=true");
    
    appendLog("📡 Simulation bringup started", "#9cdcfe");
  } else {
    appendLog("⚠️ Bringup already running", "#dcdcaa");
    load_params_btn_->setEnabled(true);
    run_robot_btn_->setEnabled(true);
    run_simulation_btn_->setEnabled(true);
  }
}

}  // namespace ar_ui

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ar_ui::ParamEditorPanel, rviz_common::Panel)
