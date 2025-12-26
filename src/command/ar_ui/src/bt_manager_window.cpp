#include "ar_ui/bt_manager_window.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QDesktopServices>
#include <QUrl>
#include <QFontDatabase>

#include <ar_projects/project_manager.h>
#include <ar_projects/yaml_parser.h>
#include <ar_projects/xml_generator.h>

namespace ar_ui
{

BTManagerWindow::BTManagerWindow(QWidget* parent)
  : QMainWindow(parent)
  , node_(nullptr)
  , ros_timer_(nullptr)
  , groot_process_(nullptr)
  , server_process_(nullptr)
  , server_ready_(false)
{
  setWindowTitle("BehaviorTree Manager");
  setMinimumSize(1000, 700);
  
  // Dark theme
  setStyleSheet(R"(
    QMainWindow { background-color: #1e1e1e; }
    QGroupBox { 
      color: #ffffff; 
      border: 1px solid #3c3c3c; 
      border-radius: 5px; 
      margin-top: 10px; 
      padding-top: 10px;
      font-weight: bold;
    }
    QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
    QComboBox, QListWidget, QTextEdit { 
      background-color: #2d2d2d; 
      color: #ffffff; 
      border: 1px solid #3c3c3c; 
      border-radius: 3px;
    }
    QPushButton { 
      background-color: #0e639c; 
      color: white; 
      border: none; 
      padding: 8px 16px; 
      border-radius: 4px;
      font-weight: bold;
    }
    QPushButton:hover { background-color: #1177bb; }
    QPushButton:pressed { background-color: #0d5a8c; }
    QPushButton:disabled { background-color: #3c3c3c; color: #808080; }
    QLabel { color: #ffffff; }
    QStatusBar { background-color: #007acc; color: white; }
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
          status_label_->setText("✓ Execution Completed");
          status_label_->setStyleSheet("color: #4ec9b0; font-size: 14px; font-weight: bold;");
          run_btn_->setEnabled(true);
          stop_btn_->setEnabled(false);
          statusBar()->showMessage("Execution completed successfully");
        } else if (msg->data.find("FAILURE") != std::string::npos) {
          status_label_->setText("✗ Execution Failed");
          status_label_->setStyleSheet("color: #f14c4c; font-size: 14px; font-weight: bold;");
          run_btn_->setEnabled(true);
          stop_btn_->setEnabled(false);
          statusBar()->showMessage("Execution failed");
        } else if (msg->data.find("STOPPED") != std::string::npos) {
          status_label_->setText("■ Stopped");
          status_label_->setStyleSheet("color: #dcdcaa; font-size: 14px; font-weight: bold;");
          run_btn_->setEnabled(true);
          stop_btn_->setEnabled(false);
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
  
  project_info_list_ = new QListWidget(this);
  project_layout->addWidget(project_info_list_);
  
  QPushButton* generate_btn = new QPushButton("📄 Generate XML", this);
  connect(generate_btn, &QPushButton::clicked, this, &BTManagerWindow::onGenerateXml);
  project_layout->addWidget(generate_btn);
  
  main_layout->addWidget(project_group_);
  
  // Center panel - XML Editor
  xml_group_ = new QGroupBox("BehaviorTree XML", this);
  QVBoxLayout* xml_layout = new QVBoxLayout(xml_group_);
  
  xml_editor_ = new QTextEdit(this);
  xml_editor_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  xml_editor_->setPlaceholderText("Generated XML will appear here...\n\nSelect a project and click 'Generate XML' to start.");
  xml_layout->addWidget(xml_editor_);
  
  main_layout->addWidget(xml_group_, 1);
  
  // Right panel - Execution
  exec_group_ = new QGroupBox("Execution", this);
  exec_group_->setMinimumWidth(280);
  exec_group_->setMaximumWidth(320);
  QVBoxLayout* exec_layout = new QVBoxLayout(exec_group_);
  
  status_label_ = new QLabel("⚪ Not Connected", this);
  status_label_->setStyleSheet("color: #808080; font-size: 14px; font-weight: bold;");
  status_label_->setAlignment(Qt::AlignCenter);
  exec_layout->addWidget(status_label_);
  
  QHBoxLayout* btn_row = new QHBoxLayout();
  run_btn_ = new QPushButton("▶ Run", this);
  run_btn_->setStyleSheet("background-color: #4CAF50;");
  stop_btn_ = new QPushButton("■ Stop", this);
  stop_btn_->setStyleSheet("background-color: #f44336;");
  stop_btn_->setEnabled(false);
  btn_row->addWidget(run_btn_);
  btn_row->addWidget(stop_btn_);
  exec_layout->addLayout(btn_row);
  
  groot_btn_ = new QPushButton("🌳 Open in Groot2", this);
  groot_btn_->setStyleSheet("background-color: #9c27b0;");
  exec_layout->addWidget(groot_btn_);
  
  QLabel* log_label = new QLabel("Execution Log:", this);
  exec_layout->addWidget(log_label);
  
  log_text_ = new QTextEdit(this);
  log_text_->setReadOnly(true);
  log_text_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  log_text_->setStyleSheet("background-color: #1a1a1a; color: #d4d4d4;");
  exec_layout->addWidget(log_text_);
  
  main_layout->addWidget(exec_group_);
  
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
  project_info_list_->clear();
  
  if (index > 0) {
    current_project_path_ = project_combo_->currentData().toString();
    current_xml_file_.clear();
    
    try {
      ar_projects::YamlParser parser;
      auto config = parser.parse(current_project_path_.toStdString());
      
      project_info_list_->addItem(QString("Name: %1").arg(QString::fromStdString(config.name)));
      project_info_list_->addItem(QString("Motions: %1").arg(config.motions.size()));
      project_info_list_->addItem(QString("Waypoints: %1").arg(config.waypoints.size()));
      project_info_list_->addItem(QString("Velocity: %1").arg(config.velocity_scaling));
      project_info_list_->addItem(QString("Acceleration: %1").arg(config.acceleration_scaling));
      
    } catch (const std::exception& e) {
      project_info_list_->addItem(QString("Error: %1").arg(e.what()));
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
  status_label_->setText("⏳ Running...");
  status_label_->setStyleSheet("color: #569cd6; font-size: 14px; font-weight: bold;");
  
  appendLog("Executing: " + QFileInfo(tree_file).fileName(), "#4ec9b0");
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

void BTManagerWindow::onClearLog()
{
  log_text_->clear();
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
  QString html = QString("<span style='color:#808080;'>[%1]</span> "
                        "<span style='color:%2;'>%3</span>")
    .arg(timestamp, color, msg.toHtmlEscaped());
  log_text_->append(html);
}

}  // namespace ar_ui
