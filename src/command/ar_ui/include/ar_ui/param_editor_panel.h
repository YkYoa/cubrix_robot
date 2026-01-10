#ifndef AR_UI__PARAM_EDITOR_PANEL_H_
#define AR_UI__PARAM_EDITOR_PANEL_H_

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QGroupBox>
#include <QScrollArea>
#include <QTextEdit>
#include <QTimer>
#include <QProcess>
#include <QTabWidget>
#include <QTableWidget>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

#include <map>
#include <vector>

namespace ar_ui
{

// Structure to hold joint parameter widgets
struct JointParamWidgets {
  QString joint_name;
  QString drive_id;
  QSpinBox* gear_ratio;
  QSpinBox* encoder_res;
  QSpinBox* encoder_offset;
  QCheckBox* log_joint;
};

// Structure to hold drive parameter widgets  
struct DriveParamWidgets {
  QString drive_id;
  QCheckBox* is_dual_axis;
  QLineEdit* driver_name;
  QSpinBox* port_id;
  std::vector<JointParamWidgets> joints;
};

/**
 * @brief RViz Panel for editing hardware parameters and triggering bringup
 */
class ParamEditorPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ParamEditorPanel(QWidget* parent = nullptr);
  ~ParamEditorPanel() override;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config& config) override;

public Q_SLOTS:
  void onLoadParams();
  void onRunRobot();
  void onRunSimulation();
  void onReloadFromFile();

private:
  void setupUi();
  void loadYamlToUi();
  void appendLog(const QString& msg, const QString& color = "#d4d4d4");
  QString generateYamlFromUi();
  void saveYamlToFile(const QString& yaml_content);

  // ROS 2
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr param_pub_;
  
  // UI Components
  QVBoxLayout* main_layout_;
  QTabWidget* tab_widget_;
  
  // Global settings tab
  QLineEdit* port_name_edit_;
  QSpinBox* driver_mode_spin_;
  QCheckBox* use_torque_offset_check_;
  
  // Drives/Joints table
  QTableWidget* joints_table_;
  
  // Action buttons
  QPushButton* reload_btn_;
  QPushButton* load_params_btn_;
  QPushButton* run_robot_btn_;
  QPushButton* run_simulation_btn_;
  
  // Status
  QLabel* status_label_;
  QTextEdit* log_text_;
  
  // Bringup process
  QProcess* bringup_process_;
  
  // Data storage
  std::vector<DriveParamWidgets> drive_widgets_;
};

}  // namespace ar_ui

#endif  // AR_UI__PARAM_EDITOR_PANEL_H_
