# AR Robot Workspace

ROS2 Humble workspace for AR robot control with EtherCAT, MoveIt, and BehaviorTree integration.

---

## Prerequisites

### Install ROS2 Humble

Follow the official installation guide: [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### Install Required ROS2 Packages

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-rclcpp \
  ros-humble-std-msgs \
  ros-humble-sensor-msgs \
  ros-humble-moveit \
  ros-humble-moveit-core \
  ros-humble-moveit-ros-planning \
  ros-humble-moveit-msgs \
  ros-humble-moveit-visual-tools \
  ros-humble-geometric-shapes \
  ros-humble-tf2-eigen \
  ros-humble-hardware-interface \
  ros-humble-pluginlib \
  ros-humble-rclcpp-lifecycle \
  ros-humble-octomap \
  ros-humble-octomap-msgs \
  ros-humble-octomap-ros \
  ros-humble-yaml-cpp-vendor \
  ros-humble-urdf \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-rviz-common \
  ros-humble-rviz-rendering \
  ros-humble-rviz-default-plugins \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-forward-command-controller \
  ros-humble-force-torque-sensor-broadcaster \
  ros-humble-ament-index-cpp \
  ros-humble-behaviortree-cpp \
  ros-humble-controller-manager \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-pilz-industrial-motion-planner
```

### Install System Dependencies

```bash
# Qt5 for UI
sudo apt-get install -y qtbase5-dev

# Build tools
sudo apt-get install -y python3-colcon-common-extensions
```

---

## Build the Workspace

```bash
cd ~/ar_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Launch Commands

### 1. Robot Bringup

```bash
# Simulation mode
ros2 launch ar_control ar_bringup.launch.py sim:=true

# Real hardware with EtherCAT
ros2 launch ar_control ar_bringup.launch.py sim:=false ecat:=true

# With UI mode
ros2 launch ar_control ar_bringup.launch.py sim:=true ui:=true
```

| Argument | Default | Description |
|----------|---------|-------------|
| `sim` | `false` | Enable simulation mode |
| `ui` | `false` | Enable UI mode |
| `ecat` | `true` | Enable IGH EtherCAT master |

### 2. MoveIt Motion Planning

```bash
ros2 launch ar_control ar_moveit.launch.py desc:=ar

# Example
ros2 launch ar_control ar_moveit.launch.py desc:=ar
```

### 3. RViz Visualization

```bash
ros2 launch ar_control ar_rviz.launch.py desc:=ar

# Example
ros2 launch ar_control ar_rviz.launch.py desc:=ar
```

### 4. BehaviorTree Executor

```bash
# Default tree
ros2 launch ar_bt bt_executor.launch.py

# Specific tree file
ros2 launch ar_bt bt_executor.launch.py tree_file:=my_motion.xml planning_group:=Arm
```

| Argument | Default | Description |
|----------|---------|-------------|
| `tree_file` | `simple_motion_test.xml` | BehaviorTree XML file |
| `planning_group` | `Arm` | Planning group name |
| `use_sim_time` | `true` | Use simulation time |

---

## Full System Launch

Run in separate terminals:

```bash
# Terminal 1: Bringup
ros2 launch ar_control ar_bringup.launch.py sim:=true

# Terminal 2: MoveIt
ros2 launch ar_control ar_moveit.launch.py desc:=ar

# Terminal 3: RViz
ros2 launch ar_control ar_rviz.launch.py desc:=ar

# Terminal 4: BehaviorTree (optional)
ros2 launch ar_bt bt_executor.launch.py
```

---

## Install Groot2 (Optional)

```bash
wget https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.6.1-linux-installer.run
chmod +x Groot2-v1.6.1-linux-installer.run
./Groot2-v1.6.1-linux-installer.run
```

Or download from: https://www.behaviortree.dev/groot
