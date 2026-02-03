#include "ar_ui/bt_manager_window.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QDesktopServices>
#include <QUrl>
#include <QFontDatabase>
#include <QRegularExpression>
#include <thread>

#include <ar_projects/project_manager.h>
#include <ar_projects/yaml_parser.h>
#include <ar_projects/xml_generator.h>
#include <ar_projects/yaml_generator.h>
#include <ar_projects/tasks/task_base.h>
#include <ar_projects/tasks/draw_rectangle.h>
#include <ar_projects/motion_executor.h>
#include <ar_projects/motion_types/move_joint.h>
#include <ar_projects/motion_types/delay_motion.h>
#include <ar_projects/motion_types/set_planner.h>
#include <ar_projects/motion_types/set_velocity.h>

namespace ar_ui
{

BTManagerWindow::BTManagerWindow(QWidget* parent)
  : QMainWindow(parent)
  , node_(nullptr)
  , ros_timer_(nullptr)
  , groot_process_(nullptr)
  , server_process_(nullptr)
  , server_ready_(false)
  , current_loop_(0)
  , target_loops_(1)
{
  setWindowTitle("BehaviorTree Manager");
  setMinimumSize(1400, 900);  // Much larger window
  resize(1600, 1000);  // Default size
  
  // Dark theme with larger fonts
  setStyleSheet(R"(
    QMainWindow { background-color: #1e1e1e; }
    QGroupBox { 
      color: #ffffff; 
      border: 1px solid #3c3c3c; 
      border-radius: 5px; 
      margin-top: 12px; 
      padding-top: 12px;
      font-weight: bold;
      font-size: 13px;
    }
    QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
    QComboBox, QListWidget { 
      background-color: #2d2d2d; 
      color: #ffffff; 
      border: 1px solid #3c3c3c; 
      border-radius: 3px;
      font-size: 12px;
      padding: 4px;
    }
    QTextEdit { 
      background-color: #1a1a1a; 
      color: #d4d4d4; 
      border: 1px solid #3c3c3c; 
      border-radius: 3px;
      font-size: 12px;
    }
    QPushButton { 
      background-color: #0e639c; 
      color: white; 
      border: none; 
      padding: 10px 20px; 
      border-radius: 4px;
      font-weight: bold;
      font-size: 13px;
    }
    QPushButton:hover { background-color: #1177bb; }
    QPushButton:pressed { background-color: #0d5a8c; }
    QPushButton:disabled { background-color: #3c3c3c; color: #808080; }
    QLabel { color: #ffffff; font-size: 12px; }
    QStatusBar { background-color: #007acc; color: white; font-size: 12px; }
    QMenuBar { background-color: #2d2d2d; color: white; }
    QMenuBar::item:selected { background-color: #3c3c3c; }
    QMenu { background-color: #2d2d2d; color: white; }
    QMenu::item:selected { background-color: #0e639c; }
  )");
  
  setupMenuBar();
  setupUi();
  
  // Find Groot2
  QStringList groot_paths = {
    "/usr/local/bin/groot2",
    QDir::homePath() + "/Groot2/bin/groot2",
    QDir::homePath() + "/Groot2.AppImage",
    "/opt/Groot2/bin/groot2",
    "/usr/bin/groot2"
  };
  
  for (const QString& path : groot_paths) {
    if (QFile::exists(path)) {
      groot_path_ = path;
      break;
    }
  }
  
  statusBar()->showMessage("Ready");
}

BTManagerWindow::~BTManagerWindow()
{
  if (groot_process_ && groot_process_->state() == QProcess::Running) {
    groot_process_->terminate();
    groot_process_->waitForFinished(1000);
  }
  if (server_process_ && server_process_->state() == QProcess::Running) {
    server_process_->terminate();
    server_process_->waitForFinished(2000);
    if (server_process_->state() == QProcess::Running) {
      server_process_->kill();
    }
  }
}

void BTManagerWindow::setRosNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  
  execution_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/ar_bt/execute_command", 10);
  
  // Use transient_local QoS to receive late messages (like READY)
  rclcpp::QoS status_qos(10);
  status_qos.transient_local();
  
  status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/ar_bt/execution_status", status_qos,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      QMetaObject::invokeMethod(this, [this, msg]() {
        QString status_msg = QString::fromStdString(msg->data);
        appendLog(status_msg, "#9cdcfe");
        
        if (msg->data.find("SUCCESS") != std::string::npos) {
          current_loop_++;
          
          // Check if we need to loop again
          if (loop_checkbox_->isChecked() && current_loop_ < target_loops_) {
            appendLog(QString("Loop %1/%2 completed, starting next...").arg(current_loop_).arg(target_loops_), "#dcdcaa");
            status_label_->setText(QString("🔄 Loop %1/%2").arg(current_loop_ + 1).arg(target_loops_));
            status_label_->setStyleSheet("color: #569cd6; font-size: 14px; font-weight: bold;");
            
            // Execute again after a brief delay
            QTimer::singleShot(500, this, [this]() {
              if (!current_xml_file_.isEmpty()) {
                executeTreeFile(current_xml_file_);
              }
            });
          } else {
            QString completed_msg = loop_checkbox_->isChecked() 
              ? QString("All %1 loops completed").arg(target_loops_)
              : "Execution Completed";
            status_label_->setText(completed_msg);
            status_label_->setStyleSheet("color: #4ec9b0; font-size: 14px; font-weight: bold;");
            run_btn_->setEnabled(true);
            stop_btn_->setEnabled(false);
            current_loop_ = 0;
            statusBar()->showMessage("Execution completed successfully");
          }
        } else if (msg->data.find("FAILURE") != std::string::npos) {
          QString fail_msg = loop_checkbox_->isChecked()
            ? QString("✗ Failed at loop %1/%2").arg(current_loop_ + 1).arg(target_loops_)
            : "✗ Execution Failed";
          status_label_->setText(fail_msg);
          status_label_->setStyleSheet("color: #f14c4c; font-size: 14px; font-weight: bold;");
          run_btn_->setEnabled(true);
          stop_btn_->setEnabled(false);
          current_loop_ = 0;
          statusBar()->showMessage("Execution failed");
        } else if (msg->data.find("STOPPED") != std::string::npos) {
          status_label_->setText("■ Stopped");
          status_label_->setStyleSheet("color: #dcdcaa; font-size: 14px; font-weight: bold;");
          run_btn_->setEnabled(true);
          stop_btn_->setEnabled(false);
          current_loop_ = 0;
          statusBar()->showMessage("Execution stopped");
        } else if (msg->data.find("READY") != std::string::npos) {
          server_ready_ = true;
          status_label_->setText("🟢 Server Ready");
          status_label_->setStyleSheet("color: #4ec9b0; font-size: 14px; font-weight: bold;");
          
          if (!pending_tree_file_.isEmpty()) {
            executeTreeFile(pending_tree_file_);
            pending_tree_file_.clear();
          }
        }
      }, Qt::QueuedConnection);
    });
  
  // Subscribe to joint states
  joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      latest_joint_state_ = *msg;
    });
  
  // Initialize TF2 buffer and listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // ROS spin timer
  ros_timer_ = new QTimer(this);
  connect(ros_timer_, &QTimer::timeout, this, &BTManagerWindow::processRosEvents);
  ros_timer_->start(50);  // 20Hz
  
  updateProjectList();
  appendLog("Connected to ROS 2", "#4ec9b0");
}

void BTManagerWindow::closeEvent(QCloseEvent* event)
{
  event->accept();
}

void BTManagerWindow::setupMenuBar()
{
  QMenuBar* menubar = menuBar();
  
  // File menu
  QMenu* file_menu = menubar->addMenu("&File");
  
  QAction* open_action = file_menu->addAction("&Open XML...");
  connect(open_action, &QAction::triggered, this, &BTManagerWindow::onOpenXmlFile);
  
  QAction* save_action = file_menu->addAction("&Save XML");
  save_action->setShortcut(QKeySequence::Save);
  connect(save_action, &QAction::triggered, this, &BTManagerWindow::onSaveXml);
  
  file_menu->addSeparator();
  
  QAction* quit_action = file_menu->addAction("&Quit");
  quit_action->setShortcut(QKeySequence::Quit);
  connect(quit_action, &QAction::triggered, this, &QMainWindow::close);
  
  // Tools menu
  QMenu* tools_menu = menubar->addMenu("&Tools");
  
  QAction* groot_action = tools_menu->addAction("Open &Groot2");
  connect(groot_action, &QAction::triggered, this, &BTManagerWindow::onOpenInGroot);
  
  QAction* clear_log_action = tools_menu->addAction("&Clear Log");
  connect(clear_log_action, &QAction::triggered, this, &BTManagerWindow::onClearLog);
  
  // Help menu
  QMenu* help_menu = menubar->addMenu("&Help");
  
  QAction* about_action = help_menu->addAction("&About");
  connect(about_action, &QAction::triggered, this, &BTManagerWindow::onAbout);
}

void BTManagerWindow::setupUi()
{
  central_widget_ = new QWidget(this);
  setCentralWidget(central_widget_);
  
  QHBoxLayout* main_layout = new QHBoxLayout(central_widget_);
  main_layout->setContentsMargins(10, 10, 10, 10);
  main_layout->setSpacing(10);
  
  // Left panel - Projects
  project_group_ = new QGroupBox("Projects", this);
  project_group_->setMinimumWidth(250);
  project_group_->setMaximumWidth(300);
  QVBoxLayout* project_layout = new QVBoxLayout(project_group_);
  
  QHBoxLayout* combo_row = new QHBoxLayout();
  project_combo_ = new QComboBox(this);
  project_combo_->setMinimumHeight(30);
  refresh_btn_ = new QPushButton("↻", this);
  refresh_btn_->setFixedSize(30, 30);
  refresh_btn_->setToolTip("Refresh projects");
  combo_row->addWidget(project_combo_);
  combo_row->addWidget(refresh_btn_);
  project_layout->addLayout(combo_row);
  
  sequence_list_ = new QListWidget(this);
  sequence_list_->setStyleSheet(
    "QListWidget { background-color: #2d2d2d; border: 1px solid #3c3c3c; }"
    "QListWidget::item { border-bottom: 1px solid #3e3e3e; padding: 0px; }"
    "QListWidget::item:selected { background-color: #383838; }"
  );
  project_layout->addWidget(sequence_list_);
  
  QPushButton* generate_btn = new QPushButton("📄 Generate YAML", this);
  connect(generate_btn, &QPushButton::clicked, this, &BTManagerWindow::onGenerateYaml);
  project_layout->addWidget(generate_btn);

  QPushButton* reload_btn = new QPushButton("🔄 Reload YAML", this);
  reload_btn->setStyleSheet("background-color: #2d7d46;");
  connect(reload_btn, &QPushButton::clicked, this, &BTManagerWindow::onReloadYaml);
  project_layout->addWidget(reload_btn);

  QPushButton* get_state_btn = new QPushButton("🤖 Get Robot State", this);
  get_state_btn->setStyleSheet("background-color: #6b5b95;");
  connect(get_state_btn, &QPushButton::clicked, this, &BTManagerWindow::onGetRobotState);
  project_layout->addWidget(get_state_btn);

  main_layout->addWidget(project_group_);
  
  // Center panel - YAML Editor
  xml_group_ = new QGroupBox("Project YAML", this);
  QVBoxLayout* xml_layout = new QVBoxLayout(xml_group_);
  
  xml_editor_ = new QTextEdit(this);
  xml_editor_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  xml_editor_->setPlaceholderText("Generated YAML will appear here...\n\nSelect a project and click 'Generate YAML' to start.");
  xml_layout->addWidget(xml_editor_);
  
  main_layout->addWidget(xml_group_, 1);
  
  // Right panel - Execution (much wider now!)
  exec_group_ = new QGroupBox("Execution", this);
  exec_group_->setMinimumWidth(550);
  exec_group_->setMaximumWidth(800);
  QVBoxLayout* exec_layout = new QVBoxLayout(exec_group_);
  
  status_label_ = new QLabel("⚪ Not Connected", this);
  status_label_->setStyleSheet("color: #808080; font-size: 16px; font-weight: bold;");
  status_label_->setAlignment(Qt::AlignCenter);
  status_label_->setMinimumHeight(30);
  exec_layout->addWidget(status_label_);
  
  QHBoxLayout* btn_row = new QHBoxLayout();
  run_btn_ = new QPushButton("▶ Run", this);
  run_btn_->setStyleSheet("background-color: #4CAF50; font-size: 14px; padding: 12px 24px;");
  run_btn_->setMinimumHeight(45);
  stop_btn_ = new QPushButton("■ Stop", this);
  stop_btn_->setStyleSheet("background-color: #f44336; font-size: 14px; padding: 12px 24px;");
  stop_btn_->setMinimumHeight(45);
  stop_btn_->setEnabled(false);
  btn_row->addWidget(run_btn_);
  btn_row->addWidget(stop_btn_);
  exec_layout->addLayout(btn_row);
  
  groot_btn_ = new QPushButton("🌳 Open in Groot2", this);
  groot_btn_->setStyleSheet("background-color: #9c27b0; font-size: 13px;");
  groot_btn_->setMinimumHeight(40);
  exec_layout->addWidget(groot_btn_);
  
  // Task execution section
  task_info_label_ = new QLabel("No task configured", this);
  task_info_label_->setStyleSheet("color: #808080; font-size: 12px; padding: 5px;");
  task_info_label_->setAlignment(Qt::AlignCenter);
  exec_layout->addWidget(task_info_label_);
  
  run_task_btn_ = new QPushButton("⚡ Run Task", this);
  run_task_btn_->setStyleSheet("background-color: #ff9800; font-size: 13px;");
  run_task_btn_->setMinimumHeight(40);
  run_task_btn_->setEnabled(false);
  run_task_btn_->setToolTip("Execute the code-based task defined in this project");
  connect(run_task_btn_, &QPushButton::clicked, this, &BTManagerWindow::onRunTask);
  exec_layout->addWidget(run_task_btn_);
  
  // Loop control row
  QHBoxLayout* loop_row = new QHBoxLayout();
  loop_checkbox_ = new QCheckBox("🔄 Loop Sequence", this);
  loop_checkbox_->setStyleSheet("color: #ffffff; font-size: 12px;");
  loop_count_spinbox_ = new QSpinBox(this);
  loop_count_spinbox_->setRange(1, 1000);
  loop_count_spinbox_->setValue(1);
  loop_count_spinbox_->setPrefix("Count: ");
  loop_count_spinbox_->setStyleSheet(
    "QSpinBox { background-color: #2d2d2d; color: #ffffff; border: 1px solid #3c3c3c; "
    "border-radius: 3px; padding: 4px; font-size: 12px; min-width: 80px; }"
    "QSpinBox::up-button, QSpinBox::down-button { width: 20px; }"
  );
  loop_count_spinbox_->setEnabled(false);
  
  connect(loop_checkbox_, &QCheckBox::toggled, this, [this](bool checked) {
    loop_count_spinbox_->setEnabled(checked);
    if (checked) {
      target_loops_ = loop_count_spinbox_->value();
    } else {
      target_loops_ = 1;
    }
  });
  
  connect(loop_count_spinbox_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value) {
    target_loops_ = value;
  });
  
  loop_row->addWidget(loop_checkbox_);
  loop_row->addWidget(loop_count_spinbox_);
  loop_row->addStretch();
  exec_layout->addLayout(loop_row);
  
  QHBoxLayout* log_header_row = new QHBoxLayout();
  QLabel* log_label = new QLabel("Execution Log:", this);
  log_label->setStyleSheet("font-size: 13px; font-weight: bold; margin-top: 10px;");
  log_header_row->addWidget(log_label);
  log_header_row->addStretch();
  QPushButton* clear_log_btn = new QPushButton("🗑 Clear", this);
  clear_log_btn->setStyleSheet("background-color: #555; padding: 5px 10px; font-size: 11px;");
  connect(clear_log_btn, &QPushButton::clicked, this, &BTManagerWindow::onClearLog);
  log_header_row->addWidget(clear_log_btn);
  exec_layout->addLayout(log_header_row);

  log_text_ = new QTextEdit(this);
  log_text_->setReadOnly(true);
  QFont log_font = QFontDatabase::systemFont(QFontDatabase::FixedFont);
  log_font.setPointSize(10);  // Larger font for readability
  log_text_->setFont(log_font);
  log_text_->setStyleSheet(
    "background-color: #300a24; color: #ffffff; border: 2px solid #4c2639; padding: 8px;"
    "QScrollBar:vertical { background: #1e1e1e; width: 12px; }"
    "QScrollBar::handle:vertical { background: #555; min-height: 20px; border-radius: 6px; }"
    "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }"
  );
  log_text_->setMinimumHeight(500);  // Very tall log area
  exec_layout->addWidget(log_text_, 1);  // Stretch to fill
  
  main_layout->addWidget(exec_group_, 1);  // Give execution panel stretch priority
  
  // Connections
  connect(refresh_btn_, &QPushButton::clicked, this, &BTManagerWindow::onRefreshProjects);
  connect(project_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &BTManagerWindow::onProjectSelected);
  connect(run_btn_, &QPushButton::clicked, this, &BTManagerWindow::onRunProject);
  connect(stop_btn_, &QPushButton::clicked, this, &BTManagerWindow::onStopProject);
  connect(groot_btn_, &QPushButton::clicked, this, &BTManagerWindow::onOpenInGroot);
}

void BTManagerWindow::updateProjectList()
{
  project_combo_->clear();
  project_combo_->addItem("-- Select Project --");
  
  try {
    ar_projects::ProjectManager manager;
    auto projects = manager.listProjects();
    
    for (const auto& project : projects) {
      QString display = QString::fromStdString(project.name);
      if (!project.description.empty()) {
        display += " - " + QString::fromStdString(project.description);
      }
      project_combo_->addItem(display, QString::fromStdString(project.filepath));
    }
    
    appendLog(QString("Loaded %1 projects").arg(projects.size()), "#9cdcfe");
  } catch (const std::exception& e) {
    appendLog(QString("Failed to load projects: %1").arg(e.what()), "#f14c4c");
  }
}

void BTManagerWindow::onRefreshProjects()
{
  updateProjectList();
  statusBar()->showMessage("Projects refreshed");
}

void BTManagerWindow::onProjectSelected(int index)
{
  sequence_list_->clear();
  current_task_name_.clear();
  task_info_label_->setText("No task configured");
  task_info_label_->setStyleSheet("color: #808080; font-size: 12px; padding: 5px;");
  run_task_btn_->setEnabled(false);
  
  if (index > 0) {
    current_project_path_ = project_combo_->currentData().toString();
    current_xml_file_.clear();
    
    try {
      ar_projects::YamlParser parser;
      auto config = parser.parse(current_project_path_.toStdString());
      
      // Check if project has a code-based task
      if (config.hasTask()) {
        current_task_name_ = QString::fromStdString(config.task_name);
        task_info_label_->setText("⚡ Task: " + current_task_name_);
        task_info_label_->setStyleSheet("color: #ff9800; font-size: 12px; font-weight: bold; padding: 5px;");
        run_task_btn_->setEnabled(true);
        appendLog("Project has code-based task: " + current_task_name_, "#ff9800");
      }
      
      int step_count = 1;
      for (const auto& motion : config.motions) {
        QWidget* widget = new QWidget();
        QHBoxLayout* layout = new QHBoxLayout(widget);
        layout->setContentsMargins(10, 10, 10, 10);
        
        // Timeline Section (Left)
        QVBoxLayout* timeline_layout = new QVBoxLayout();
        QLabel* step_num = new QLabel(QString("%1").arg(step_count++, 2, 10, QChar('0')));
        step_num->setStyleSheet("color: #606060; font-family: monospace; font-weight: bold; font-size: 14px;");
        timeline_layout->addWidget(step_num);
        timeline_layout->addStretch();
        layout->addLayout(timeline_layout);
        
        // Content Section (Right)
        QVBoxLayout* content_layout = new QVBoxLayout();
        
        // Header: Type Badge + Name
        QHBoxLayout* header = new QHBoxLayout();
        QString type_str = QString::fromStdString(motion->getType());
        QLabel* type_badge = new QLabel(type_str);
        
        QString badge_color = "#0e639c"; // Blue (default/move)
        if (type_str == "delay") badge_color = "#dcdcaa"; // Yellow
        else if (type_str.startsWith("set_")) badge_color = "#c586c0"; // Purple
        else if (type_str == "gripper") badge_color = "#ce9178"; // Orange
        
        type_badge->setStyleSheet(QString("background-color: %1; color: white; border-radius: 3px; padding: 2px 6px; font-weight: bold; font-size: 10px;").arg(badge_color));
        QLabel* name_label = new QLabel(QString::fromStdString(motion->getName()));
        name_label->setStyleSheet("color: white; font-weight: bold; font-size: 13px;");
        
        header->addWidget(type_badge);
        header->addWidget(name_label);
        header->addStretch();
        content_layout->addLayout(header);
        
        // Details
        QString details_txt = "";
        std::string type = motion->getType();
        
        if (type == "move_joint") {
          auto move = std::dynamic_pointer_cast<ar_projects::MoveJoint>(motion);
          if (move) {
            if (!move->getWaypoint().empty()) {
              details_txt = QString("Goal: <span style='color: #4ec9b0;'>%1</span>")
                .arg(QString::fromStdString(move->getWaypoint()));
            } else {
              details_txt = "Goal: <span style='color: #dcdcaa;'>Custom Joints</span>";
            }
          }
        } else if (type == "delay") {
          auto delay = std::dynamic_pointer_cast<ar_projects::DelayMotion>(motion);
          if (delay) {
            details_txt = QString("Duration: <span style='color: #dcdcaa;'>%1 ms</span>")
              .arg(delay->getDurationMs());
          }
        } else if (type == "set_planner") {
          auto planner = std::dynamic_pointer_cast<ar_projects::SetPlannerMotion>(motion);
          if (planner) {
            details_txt = QString("Pipeline: <span style='color: #569cd6;'>%1</span>, ID: <span style='color: #4ec9b0;'>%2</span>")
              .arg(QString::fromStdString(planner->getPipeline()))
              .arg(QString::fromStdString(planner->getPlannerId()));
          }
        } else if (type == "set_velocity") {
          auto vel = std::dynamic_pointer_cast<ar_projects::SetVelocityMotion>(motion);
          if (vel) {
            details_txt = QString("Factor: <span style='color: #569cd6;'>%1</span>")
              .arg(vel->getFactor());
          }
        } else if (type == "set_acceleration") {
          auto acc = std::dynamic_pointer_cast<ar_projects::SetAccelerationMotion>(motion);
          if (acc) {
            details_txt = QString("Factor: <span style='color: #569cd6;'>%1</span>")
              .arg(acc->getFactor());
          }
        } else if (type == "set_tool") {
          details_txt = "Action: <span style='color: #ce9178;'>Set Tool State</span>";
        } else {
          details_txt = "<span style='color: #808080;'>Generic Motion Step</span>";
        }
        
        QLabel* details = new QLabel(details_txt);
        details->setTextFormat(Qt::RichText);
        details->setStyleSheet("font-size: 12px; margin-top: 4px;");
        content_layout->addWidget(details);
        
        layout->addLayout(content_layout);
        layout->addStretch();
        
        // Add to list
        QListWidgetItem* item = new QListWidgetItem(sequence_list_);
        item->setSizeHint(widget->sizeHint());
        sequence_list_->setItemWidget(item, widget);
      }
      
    } catch (const std::exception& e) {
      QListWidgetItem* item = new QListWidgetItem("Error loading project", sequence_list_);
      item->setForeground(QBrush(QColor("#f14c4c")));
    }
  }
}

QString BTManagerWindow::generateProjectXml(const QString& project_path)
{
  try {
    ar_projects::YamlParser parser;
    auto config = parser.parse(project_path.toStdString());
    
    ar_projects::XmlGenerator generator;
    std::string xml = generator.generate(config);
    
    QString output_path = "/tmp/ar_project_" + 
      QString::fromStdString(config.name) + ".xml";
    
    QFile file(output_path);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
      QTextStream out(&file);
      out << QString::fromStdString(xml);
      file.close();
      
      xml_editor_->setPlainText(QString::fromStdString(xml));
      return output_path;
    }
  } catch (const std::exception& e) {
    appendLog(QString("Generation error: %1").arg(e.what()), "#f14c4c");
  }
  return "";
}

void BTManagerWindow::onGenerateXml()
{
  if (current_project_path_.isEmpty()) {
    QMessageBox::warning(this, "No Project", "Please select a project first.");
    return;
  }
  
  QString xml_path = generateProjectXml(current_project_path_);
  
  if (!xml_path.isEmpty()) {
    current_xml_file_ = xml_path;
    appendLog("Generated: " + xml_path, "#4ec9b0");
    statusBar()->showMessage("XML generated: " + xml_path);
  } else {
    QMessageBox::critical(this, "Error", "Failed to generate XML.");
  }
}

void BTManagerWindow::onGenerateYaml()
{
  if (current_project_path_.isEmpty()) {
    QMessageBox::warning(this, "No Project", "Please select a project first.");
    return;
  }
  
  try {
    ar_projects::YamlParser parser;
    auto config = parser.parse(current_project_path_.toStdString());
    
    ar_projects::YamlGenerator generator;
    std::string yaml = generator.generate(config);
    
    // Display in editor
    xml_editor_->setPlainText(QString::fromStdString(yaml));
    
    // Save to file
    QString output_path = "/tmp/ar_project_" + 
      QString::fromStdString(config.name) + ".yaml";
    
    QFile file(output_path);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
      QTextStream out(&file);
      out << QString::fromStdString(yaml);
      file.close();
      
      appendLog("Generated YAML: " + output_path, "#4ec9b0");
      statusBar()->showMessage("YAML generated: " + output_path);
    }
  } catch (const std::exception& e) {
    appendLog(QString("YAML generation error: %1").arg(e.what()), "#f14c4c");
    QMessageBox::critical(this, "Error", QString("Failed to generate YAML: %1").arg(e.what()));
  }
}

void BTManagerWindow::onOpenInGroot()
{
  if (current_xml_file_.isEmpty() && !current_project_path_.isEmpty()) {
    onGenerateXml();
  }
  
  if (!groot_path_.isEmpty() && QFile::exists(groot_path_)) {
    if (!groot_process_) {
      groot_process_ = new QProcess(this);
    }
    
    if (groot_process_->state() != QProcess::Running) {
      QStringList args;
      if (!current_xml_file_.isEmpty()) {
        args << current_xml_file_;
      }
      groot_process_->start(groot_path_, args);
      appendLog("Launching Groot2...", "#9cdcfe");
      statusBar()->showMessage("Opening Groot2");
    } else {
      statusBar()->showMessage("Groot2 is already running");
    }
  } else {
    QMessageBox::warning(this, "Groot Not Found",
      "Groot2 not found.\n\nDownload from: https://www.behaviortree.dev/groot");
  }
}

void BTManagerWindow::startBtServer()
{
  if (!server_process_) {
    server_process_ = new QProcess(this);
    
    connect(server_process_, &QProcess::readyReadStandardOutput, this, [this]() {
      QString output = server_process_->readAllStandardOutput();
      for (const QString& line : output.split('\n', Qt::SkipEmptyParts)) {
        appendLog("[server] " + line.trimmed(), "#808080");
      }
    });
    
    connect(server_process_, &QProcess::readyReadStandardError, this, [this]() {
      QString output = server_process_->readAllStandardError();
      for (const QString& line : output.split('\n', Qt::SkipEmptyParts)) {
        appendLog("[server] " + line.trimmed(), "#dcdcaa");
      }
    });
  }
  
  if (server_process_->state() != QProcess::Running) {
    status_label_->setText("🟡 Starting Server...");
    status_label_->setStyleSheet("color: #dcdcaa; font-size: 14px; font-weight: bold;");
    
    server_process_->start("ros2", QStringList() 
      << "run" << "ar_bt" << "bt_server_node"
      << "--ros-args" << "-p" << "planning_group:=Arm");
    
    appendLog("Starting BT server...", "#9cdcfe");
    statusBar()->showMessage("Starting BT server...");
  }
}

void BTManagerWindow::onRunProject()
{
  if (current_xml_file_.isEmpty()) {
    if (current_project_path_.isEmpty()) {
      QMessageBox::warning(this, "No Project", "Please select a project first.");
      return;
    }
    current_xml_file_ = generateProjectXml(current_project_path_);
    if (current_xml_file_.isEmpty()) {
      return;
    }
  }
  
  // Auto-start server if not running
  if (!server_process_ || server_process_->state() != QProcess::Running) {
    pending_tree_file_ = current_xml_file_;
    appendLog("Auto-starting BT server...", "#9cdcfe");
    startBtServer();
    
    // Wait for server to be ready (5 seconds)
    status_label_->setText("⏳ Starting server...");
    status_label_->setStyleSheet("color: #dcdcaa; font-size: 14px; font-weight: bold;");
    run_btn_->setEnabled(false);
    
    QTimer::singleShot(5000, this, [this]() {
      if (!pending_tree_file_.isEmpty()) {
        appendLog("Server should be ready, executing tree...", "#9cdcfe");
        executeTreeFile(pending_tree_file_);
        pending_tree_file_.clear();
      }
    });
  } else {
    executeTreeFile(current_xml_file_);
  }
}

void BTManagerWindow::executeTreeFile(const QString& tree_file)
{
  if (!execution_pub_) return;
  
  std_msgs::msg::String msg;
  msg.data = tree_file.toStdString();
  execution_pub_->publish(msg);
  
  run_btn_->setEnabled(false);
  stop_btn_->setEnabled(true);
  
  if (loop_checkbox_->isChecked() && target_loops_ > 1) {
    status_label_->setText(QString("⏳ Loop %1/%2").arg(current_loop_ + 1).arg(target_loops_));
  } else {
    status_label_->setText("⏳ Running...");
  }
  status_label_->setStyleSheet("color: #569cd6; font-size: 14px; font-weight: bold;");
  
  QString log_msg = loop_checkbox_->isChecked() && target_loops_ > 1
    ? QString("Executing: %1 (Loop %2/%3)").arg(QFileInfo(tree_file).fileName()).arg(current_loop_ + 1).arg(target_loops_)
    : "Executing: " + QFileInfo(tree_file).fileName();
  appendLog(log_msg, "#4ec9b0");
  statusBar()->showMessage("Executing behavior tree...");
}

void BTManagerWindow::onStopProject()
{
  if (!execution_pub_) return;
  
  std_msgs::msg::String msg;
  msg.data = "STOP";
  execution_pub_->publish(msg);
  
  run_btn_->setEnabled(true);
  stop_btn_->setEnabled(false);
  pending_tree_file_.clear();
  current_loop_ = 0;  // Reset loop counter on stop
  
  appendLog("Stop requested", "#dcdcaa");
  statusBar()->showMessage("Execution stopped");
}

void BTManagerWindow::onOpenXmlFile()
{
  QString file = QFileDialog::getOpenFileName(this, "Open XML File", 
    "/tmp", "XML Files (*.xml);;All Files (*)");
  
  if (!file.isEmpty()) {
    QFile f(file);
    if (f.open(QIODevice::ReadOnly | QIODevice::Text)) {
      xml_editor_->setPlainText(f.readAll());
      current_xml_file_ = file;
      f.close();
      appendLog("Opened: " + file, "#9cdcfe");
    }
  }
}

void BTManagerWindow::onSaveXml()
{
  if (current_xml_file_.isEmpty()) {
    current_xml_file_ = QFileDialog::getSaveFileName(this, "Save XML File",
      "/tmp/behavior_tree.xml", "XML Files (*.xml)");
  }
  
  if (!current_xml_file_.isEmpty()) {
    QFile file(current_xml_file_);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
      QTextStream out(&file);
      out << xml_editor_->toPlainText();
      file.close();
      appendLog("Saved: " + current_xml_file_, "#4ec9b0");
      statusBar()->showMessage("File saved");
    }
  }
}

void BTManagerWindow::onReloadYaml()
{
  if (current_project_path_.isEmpty()) {
    QMessageBox::warning(this, "No Project", "Please select a project first.");
    return;
  }
  
  // Re-trigger onProjectSelected to refresh the sequence view
  int index = project_combo_->currentIndex();
  onProjectSelected(index);
  
  appendLog("Reloaded YAML: " + current_project_path_, "#4ec9b0");
  statusBar()->showMessage("YAML reloaded");
}

void BTManagerWindow::onClearLog()
{
  log_text_->clear();
}

void BTManagerWindow::onGetRobotState()
{
  if (latest_joint_state_.name.empty()) {
    QMessageBox::warning(this, "No Data", "No joint state data received yet.\nMake sure /joint_states topic is being published.");
    return;
  }
  
  // ===== YAML Waypoint Format Output =====
  QString yaml_output = "<pre style='color: #ffffff; background-color: #1e1e1e; padding: 10px; border-radius: 5px;'>";
  yaml_output += "<span style='color: #ce9178;'>current_state:</span>\n";
  yaml_output += "  <span style='color: #9cdcfe;'>joints:</span> [";
  
  for (size_t i = 0; i < latest_joint_state_.position.size(); ++i) {
    yaml_output += QString::number(latest_joint_state_.position[i], 'f', 4);
    if (i < latest_joint_state_.position.size() - 1) yaml_output += ", ";
  }
  yaml_output += "]\n";
  yaml_output += "</pre>";
  
  log_text_->append(yaml_output);
  
  // ===== End Effector Pose (TF2 Lookup) =====
  try {
    // Try common end effector frame names
    // Try common end effector frame names (your robot uses Arm_eelink and Base)
    std::vector<std::string> ee_frames = {"Arm_eelink", "Arm_endlink", "tool0", "ee_link", "end_effector", "link_6", "gripper"};
    std::string base_frame = "Base";
    
    geometry_msgs::msg::TransformStamped transform;
    bool found = false;
    std::string used_frame;
    
    for (const auto& ee_frame : ee_frames) {
      if (tf_buffer_->canTransform(base_frame, ee_frame, tf2::TimePointZero, tf2::durationFromSec(0.1))) {
        transform = tf_buffer_->lookupTransform(base_frame, ee_frame, tf2::TimePointZero);
        found = true;
        used_frame = ee_frame;
        break;
      }
    }
    
    if (found) {
      QString pose_output = "<pre style='color: #ffffff; background-color: #1e1e1e; padding: 10px; border-radius: 5px;'>";
      pose_output += "<span style='color: #569cd6;'># End Effector Pose (" + QString::fromStdString(used_frame) + "):</span>\n";
      pose_output += QString("<span style='color: #ce9178;'>position:</span>\n");
      pose_output += QString("  <span style='color: #9cdcfe;'>x:</span> %1\n").arg(transform.transform.translation.x, 0, 'f', 6);
      pose_output += QString("  <span style='color: #9cdcfe;'>y:</span> %1\n").arg(transform.transform.translation.y, 0, 'f', 6);
      pose_output += QString("  <span style='color: #9cdcfe;'>z:</span> %1\n").arg(transform.transform.translation.z, 0, 'f', 6);
      pose_output += QString("<span style='color: #ce9178;'>orientation:</span>\n");
      pose_output += QString("  <span style='color: #9cdcfe;'>qx:</span> %1\n").arg(transform.transform.rotation.x, 0, 'f', 6);
      pose_output += QString("  <span style='color: #9cdcfe;'>qy:</span> %1\n").arg(transform.transform.rotation.y, 0, 'f', 6);
      pose_output += QString("  <span style='color: #9cdcfe;'>qz:</span> %1\n").arg(transform.transform.rotation.z, 0, 'f', 6);
      pose_output += QString("  <span style='color: #9cdcfe;'>qw:</span> %1\n").arg(transform.transform.rotation.w, 0, 'f', 6);
      pose_output += "</pre>";
      
      log_text_->append(pose_output);
    } else {
      appendLog("End effector TF not found (tried: tool0, ee_link, end_effector, link_6, gripper)", "#dcdcaa");
    }
  } catch (const tf2::TransformException& e) {
    appendLog(QString("TF Error: %1").arg(e.what()), "#f14c4c");
  }
  
  appendLog("Retrieved robot state (" + QString::number(latest_joint_state_.name.size()) + " joints)", "#9cdcfe");
  statusBar()->showMessage("Robot state retrieved");
}

void BTManagerWindow::onAbout()
{
  QMessageBox::about(this, "About BehaviorTree Manager",
    "<h2>BehaviorTree Manager</h2>"
    "<p>Version 1.0</p>"
    "<p>A tool for managing and executing BehaviorTree projects.</p>"
    "<p>Part of the AR Robot Control System.</p>");
}

void BTManagerWindow::processRosEvents()
{
  if (node_) {
    rclcpp::spin_some(node_);
  }
}

void BTManagerWindow::appendLog(const QString& msg, const QString& color)
{
  QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
  
  // Convert ANSI escape codes to HTML colors
  QString processed = msg;
  
  // First, strip any remaining escape characters (the  symbol)
  processed.remove(QChar('\x1b'));
  
  // ANSI color code mappings (now without escape char)
  // [30m=black, [31m=red, [32m=green, [33m=yellow, [34m=blue, [35m=magenta, [36m=cyan, [37m=white
  // [0m = reset
  processed.replace("[36m", "<span style='color:#56b6c2;'>");  // Cyan (IDLE)
  processed.replace("[32m", "<span style='color:#98c379;'>");  // Green (SUCCESS)
  processed.replace("[33m", "<span style='color:#e5c07b;'>");  // Yellow (RUNNING)
  processed.replace("[31m", "<span style='color:#e06c75;'>");  // Red (FAILURE)
  processed.replace("[35m", "<span style='color:#c678dd;'>");  // Magenta
  processed.replace("[34m", "<span style='color:#61afef;'>");  // Blue
  processed.replace("[0m", "</span>");  // Reset
  
  QString html = QString("<span style='color:#5c6370;'>[%1]</span> "
                        "<span style='color:%2;'>%3</span>")
    .arg(timestamp, color, processed);
  log_text_->append(html);
}

void BTManagerWindow::onRunTask()
{
  if (current_task_name_.isEmpty()) {
    QMessageBox::warning(this, "No Task", "This project has no code-based task configured.");
    return;
  }

  if (!node_) {
    QMessageBox::warning(this, "Not Connected", "ROS2 node not initialized.");
    return;
  }

  appendLog("Starting task: " + current_task_name_, "#ff9800");
  status_label_->setText("⏳ Running Task...");
  status_label_->setStyleSheet("color: #ff9800; font-size: 14px; font-weight: bold;");
  run_task_btn_->setEnabled(false);

  // Parse project config to get task params and waypoints
  ar_projects::YamlParser parser;
  auto config = parser.parse(current_project_path_.toStdString());

  // Create task from registry
  auto task = ar_projects::TaskRegistry::instance().create(current_task_name_.toStdString());
  if (!task) {
    appendLog("ERROR: Task not found in registry: " + current_task_name_, "#f14c4c");
    status_label_->setText("✗ Task Not Found");
    status_label_->setStyleSheet("color: #f14c4c; font-size: 14px; font-weight: bold;");
    run_task_btn_->setEnabled(true);
    return;
  }

  // Configure task with params
  if (!task->configure(config.task_params)) {
    appendLog("ERROR: Failed to configure task", "#f14c4c");
    status_label_->setText("✗ Configuration Failed");
    status_label_->setStyleSheet("color: #f14c4c; font-size: 14px; font-weight: bold;");
    run_task_btn_->setEnabled(true);
    return;
  }

  appendLog("Executing task: " + QString::fromStdString(task->description()), "#9cdcfe");

  // Run task in a background thread so ROS2 can continue spinning
  // MoveIt2 action clients need the node to spin to receive results
  std::thread task_thread([this, task, config]() {
    // Create motion executor in the task thread
    auto executor = std::make_shared<ar_projects::MotionExecutor>(node_, "Arm");
    executor->setWaypoints(config.waypoints);
    executor->setVelocityScaling(config.velocity_scaling);
    executor->setAccelerationScaling(config.acceleration_scaling);

    bool success = task->execute(*executor);

    // Update UI from main thread using QMetaObject::invokeMethod
    QMetaObject::invokeMethod(this, [this, success]() {
      if (success) {
        appendLog("Task completed successfully!", "#4ec9b0");
        status_label_->setText("✓ Task Completed");
        status_label_->setStyleSheet("color: #4ec9b0; font-size: 14px; font-weight: bold;");
        statusBar()->showMessage("Task execution completed");
      } else {
        appendLog("Task execution failed!", "#f14c4c");
        status_label_->setText("✗ Task Failed");
        status_label_->setStyleSheet("color: #f14c4c; font-size: 14px; font-weight: bold;");
        statusBar()->showMessage("Task execution failed");
      }
      run_task_btn_->setEnabled(true);
    }, Qt::QueuedConnection);
  });

  task_thread.detach();  // Let it run independently
}

}  // namespace ar_ui

