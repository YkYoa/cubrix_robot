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
  /**
   * @brief Constructor
   * @param parent Parent widget
   */
  explicit ParamEditorPanel(QWidget* parent = nullptr);
  
  /**
   * @brief Destructor
   */
  ~ParamEditorPanel() override;

  /**
   * @brief Initialize the panel (RViz interface)
   */
  void onInitialize() override;
  
  /**
   * @brief Save panel configuration
   * @param config RViz config to save to
   */
  void save(rviz_common::Config config) const override;
  
  /**
   * @brief Load panel configuration
   * @param config RViz config to load from
   */
  void load(const rviz_common::Config& config) override;

public Q_SLOTS:
  /**
   * @brief Load parameters to hardware interface
   */
  void onLoadParams();
  
  /**
   * @brief Run robot bringup (real hardware)
   */
  void onRunRobot();
  
  /**
   * @brief Run robot bringup (simulation)
   */
  void onRunSimulation();
  
  /**
   * @brief Reload parameters from YAML file
   */
  void onReloadFromFile();

private:
  /**
   * @brief Setup UI widgets
   */
  void setupUi();
  
  /**
   * @brief Load YAML configuration into UI widgets
   */
  void loadYamlToUi();
  
  /**
   * @brief Append message to log with color
   * @param msg Message to append
   * @param color HTML color (default: "#d4d4d4")
   */
  void appendLog(const QString& msg, const QString& color = "#d4d4d4");
  
  /**
   * @brief Generate YAML from UI widget values
   * @return YAML string
   */
  QString generateYamlFromUi();
  
  /**
   * @brief Save YAML content to file
   * @param yaml_content YAML content to save
   */
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
