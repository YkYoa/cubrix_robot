# AR Robot Workspace

ROS2 Humble workspace for AR robot control with EtherCAT, MoveIt, and BehaviorTree integration.

---

## Table of Contents

- [Prerequisites](#prerequisites)
- [Build the Workspace](#build-the-workspace)
- [Packages Overview](#packages-overview)
- [Quick Start](#quick-start)
- [Launch Commands](#launch-commands)
- [Documentation](#documentation)
- [Troubleshooting](#troubleshooting)

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
  ros-humble-behaviortree-cpp-v3 \
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

# Doxygen for documentation (optional)
sudo apt-get install -y doxygen graphviz
```

### Install Groot2 (Optional but Recommended)

```bash
wget https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.6.1-linux-installer.run
chmod +x Groot2-v1.6.1-linux-installer.run
./Groot2-v1.6.1-linux-installer.run
```

Or download from: https://www.behaviortree.dev/groot

---

## Build the Workspace

```bash
cd ~/ar_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Packages Overview

### Command & Control

#### `ar_bt` - Behavior Tree Integration
BehaviorTree.CPP v3 integration for robot motion sequencing with visual programming via Groot2.

**Features:**
- Action Nodes: `MoveToJoint`, `PlanToJoint`, `SetPlanner`, `SetBlendRadius`, `SetVelocityScaling`, `SetAccelerationScaling`
- Condition Nodes: `IsAtJoint`, `IsPlanValid`, `IsRobotReady`
- Modular SubTrees: Primitives (`go_home`, `setup_motion`) and Skills (`pick_object`, `place_object`)
- BT Server: Background service for UI integration

**Quick Start:**
```bash
# Run BT Executor (one-shot)
ros2 run ar_bt bt_executor_node --ros-args -p tree_file:=pick_place.xml

# Run BT Server (for UI)
ros2 run ar_bt bt_server_node --ros-args -p planning_group:=Arm
```

#### `ar_control` - Robot Control
Core robot control system with hardware interface, drive control, joint control, and ROS2 control integration.

#### `ar_motion_planner` - Motion Planner
High-level motion planning functionality.

#### `ar_planning_interface` - Planning Interface
MoveIt planning interface wrapper for easier integration.

#### `ar_projects` - Project Manager
YAML-configurable motion sequence generator for BehaviorTree XML files.

**Project Structure:**
```
ar_projects/
├── config/
│   └── defaults.yaml        # Global defaults
└── projects/
    ├── pick_place/
    │   └── config.yaml      # Project configuration
    └── demo_sequence/
        └── config.yaml
```

**Creating a New Project:**
1. Create folder: `projects/my_project/`
2. Create `config.yaml`:
```yaml
project:
  name: "my_project"
  description: "My custom motion"
  motion_types:
    - move_joint
    - delay

defaults:
  velocity_scaling: 0.5
  planner:
    pipeline: "pilz"
    planner_id: "PTP"

waypoints:
  home:
    joints: [0, 0, 0, 0, 0, 0]
  target:
    joints: [0.5, 0.3, -0.5, 0, 0, 0]

sequence:
  - type: "move_joint"
    waypoint: "home"
    name: "GoHome"
  - type: "move_joint"
    waypoint: "target"
    name: "GoToTarget"
```

**Available Motion Types:**
| Type | Description | Parameters |
|------|-------------|------------|
| `move_joint` | Move to joint config | `joints: [...]` or `waypoint: "name"` |
| `delay` | Wait for duration | `duration_ms: 500` |
| `set_planner` | Configure planner | `pipeline`, `planner_id` |
| `set_velocity` | Set velocity scaling | `factor: 0.5` |
| `set_acceleration` | Set acceleration | `factor: 0.5` |

#### `ar_scene` - Scene Manager
3D scene and mesh management.

#### `ar_ui` - User Interface
Qt-based RViz panel for managing and executing BehaviorTree projects.

**Features:**
- Project Browser: Load and select YAML-configured motion projects
- XML Generation: Auto-generate BehaviorTree XML from YAML config
- One-Click Execution: Run/Stop with real-time status
- Live Execution Log: Terminal-style log with colored output
- Groot2 Integration: Open trees in visual editor

**Usage:**
1. Launch RViz: `ros2 launch ar_control ar_rviz.launch.py desc:=<robot>`
2. The **BehaviorTree** panel appears in the sidebar
3. Select a project from the dropdown
4. Click **🌳 Open in Groot** to edit/view the tree
5. Click **▶ Run Project** to execute

### Communication

#### `ar_comm` - Communication Library
ROS2 communication utilities and base classes for topic communication.

#### `ar_serial` - Serial Communication
Serial port interface for device communication.

#### `devices` - Device Management
Client/server architecture for device management.

#### `master` - EtherCAT Master
EtherCAT master implementation using IGH EtherCAT library.

### Utilities

#### `ar_common` - Common Definitions
Shared definitions for joints, links, poses, and planners.

#### `ar_computation` - Computation
Mathematical utilities, conversions, and data objects.

#### `ar_utils` - Utilities
Common utilities for logging, serialization, and general helpers.

---

## Quick Start

### Full System Launch

Run in separate terminals:

```bash
# Terminal 1: Bringup
ros2 launch ar_control ar_bringup.launch.py sim:=true

# Terminal 2: MoveIt
ros2 launch ar_control ar_moveit.launch.py desc:=ar

# Terminal 3: RViz with UI
ros2 launch ar_control ar_rviz.launch.py desc:=ar

# Terminal 4: BehaviorTree (optional)
ros2 launch ar_bt bt_executor.launch.py
```

### Using the UI Panel

1. Launch RViz: `ros2 launch ar_control ar_rviz.launch.py desc:=ar`
2. In RViz: **Panels → Add New Panel → ar_ui/BTManagerPanel**
3. Select a project from the dropdown
4. Click **Generate XML** to convert YAML to BehaviorTree XML
5. Click **▶ Run** to execute the motion sequence
6. Monitor execution in the live log panel

---

## Launch Commands

### Robot Bringup

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

### MoveIt Motion Planning

```bash
ros2 launch ar_control ar_moveit.launch.py desc:=ar
```

### RViz Visualization

```bash
ros2 launch ar_control ar_rviz.launch.py desc:=ar
```

### BehaviorTree Executor

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

## Documentation

### 📚 API Documentation

The complete API documentation is automatically built and deployed using GitHub Actions.

- **🔗 View Online**: [Download the Doxygen docs](https://github.com/[your-username]/[repo-name]/actions) (from latest workflow run artifacts)
- **Browse locally**: Generate locally with `doxygen Doxyfile` in the `src/` directory

### 🔧 Generating Documentation Locally

```bash
cd ~/ar_ws/src
doxygen Doxyfile
```

### Viewing the Documentation

After generating the documentation, you can view it in several ways:

#### Method 1: Direct File Path (Simplest)

Open the main documentation file directly in your browser:

```bash
# Using file path
firefox ~/ar_ws/src/docs/html/index.html
# or
xdg-open ~/ar_ws/src/docs/html/index.html
```

Or simply navigate to `/home/hans/ar_ws/src/docs/html/index.html` in your file manager and double-click `index.html`.

#### Method 2: Local Web Server (Recommended)

For a better experience (especially for testing GitHub Pages locally), use a local web server:

```bash
cd ~/ar_ws/src/docs/html
python3 -m http.server 8000
```

Then open **http://localhost:8000** in your browser.

Press `Ctrl+C` to stop the server when done.

#### Method 3: Using a Different Port

If port 8000 is in use, specify a different port:

```bash
python3 -m http.server 8080  # Use port 8080 instead
```

Then open **http://localhost:8080** in your browser.

#### Documentation Location

The generated HTML documentation is located at:
```
~/ar_ws/src/docs/html/
```

The main entry point is:
```
~/ar_ws/src/docs/html/index.html
```

#### What You'll Find

The documentation includes:
- **Overview**: Complete overview of all packages organized by functionality
- **Classes**: Detailed class documentation with methods, parameters, and return values
- **Files**: Source file listings with code documentation
- **Search**: Full-text search across all documentation
- **Navigation**: Easy navigation between related components
- **Modules**: Organized by Command & Control, Communication, and Utilities

### Documentation Structure

The documentation is organized into three main modules:

- **Command & Control**: Behavior Trees, Robot Control, Motion Planning, Project Manager, Scene Manager, User Interface
- **Communication**: Communication libraries, Serial, Device Management, EtherCAT Master
- **Utilities**: Common definitions, Computation, Utilities

Each module contains detailed documentation for:
- Classes and their methods
- Function parameters and return values
- File structure and organization
- Usage examples and code snippets

### Automated Documentation Build

This repository uses **GitHub Actions** to automatically build and deploy Doxygen documentation, avoiding the need to commit 1000+ generated HTML files.

#### How It Works

1. **Source Files Only**: Only these files are committed to git:
   - `src/Doxyfile` - Doxygen configuration
   - `src/mainpage.dox` - Main documentation page
   - Source code files (`.cpp`, `.h`, `.hpp`) with Doxygen comments

2. **Generated Files Ignored**: All generated HTML/PDF/XML files are excluded via `.gitignore`:
   - `src/docs/html/` - Generated HTML documentation
   - `src/docs/latex/` - Generated LaTeX documentation
   - `src/docs/xml/` - Generated XML documentation

3. **Automatic Build**: GitHub Actions workflow (`.github/workflows/docs.yml`) automatically:
   - Builds documentation on every push to `main`/`master`
   - Deploys to GitHub Pages
   - Makes artifacts available for download

#### Setup Instructions

##### 1. Enable GitHub Pages

1. Go to your repository on GitHub
2. Navigate to **Settings → Pages**
3. Under "Build and deployment":
   - Source: Select **"GitHub Actions"**
4. Save

##### 2. Push the Workflow

The workflow file (`.github/workflows/docs.yml`) is already created. Just commit and push:

```bash
cd ~/ar_ws
git add .github/workflows/docs.yml .gitignore src/Doxyfile src/mainpage.dox
git commit -m "Add automated documentation build with GitHub Actions"
git push
```

##### 3. Access Documentation

After the first workflow run completes:

- **GitHub Pages URL**: `https://[username].github.io/[repo-name]/`
- **Download Artifacts**: Go to **Actions** tab → Latest workflow run → Download artifacts

#### Workflow Triggers

The documentation rebuilds automatically when:
- Code is pushed to `main` or `master` branch
- Files matching these patterns change:
  - `src/**/*.cpp`
  - `src/**/*.h`
  - `src/**/*.hpp`
  - `src/Doxyfile`
  - `src/mainpage.dox`
- Manual trigger via "Run workflow" button

### 📝 Contributing

When adding new code, please document it using Doxygen comments:

```cpp
/**
 * @brief Brief description of the function
 * 
 * @param param1 Description of parameter 1
 * @param param2 Description of parameter 2
 * @return Description of return value
 */
int myFunction(int param1, double param2);
```

---

## Troubleshooting

### Build Issues

**"behaviortree_cpp not found"**
```bash
sudo apt install ros-humble-behaviortree-cpp-v3
```

**"Package not found"**
- Ensure all prerequisites are installed
- Run `colcon build` from the workspace root
- Source the workspace: `source install/setup.bash`

### Runtime Issues

**"Not Connected" (UI)**
- Ensure ROS2 is sourced: `source install/setup.bash`
- Check if MoveIt is running
- Verify ROS2 daemon: `ros2 node list`

**Projects not showing (UI)**
- Verify YAML files in `src/command/ar_projects/projects/`
- Click refresh button in the UI
- Check file permissions

**Execution fails**
- Ensure MoveIt is running
- Check joint count matches your robot
- Verify `planning_group` parameter
- Check BT Server logs in terminal

### Quick Kill for ROS Processes

If a launch gets stuck, add this alias to your shell (e.g., `~/.bashrc`):

```bash
alias ros_kill_all="pkill -f 'ar_control.*launch|move_group|rviz2|bt_manager|param_editor'"
```

Then run `source ~/.bashrc` (or start a new shell) and call `ros_kill_all`.

---

## Resources

- [BehaviorTree.CPP](https://www.behaviortree.dev/)
- [Groot2 Editor](https://www.behaviortree.dev/groot)
- [MoveIt Documentation](https://moveit.picknik.ai/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

---
