# AR UI - BehaviorTree Manager

## Overview

Qt-based RViz panel for managing and executing BehaviorTree projects. Provides a visual interface for loading YAML projects, generating XML trees, and executing robot motions.

## Features

- ✅ **Project Browser**: Load and select YAML-configured motion projects
- ✅ **XML Generation**: Auto-generate BehaviorTree XML from YAML config
- ✅ **One-Click Execution**: Run/Stop with real-time status
- ✅ **Live Execution Log**: Terminal-style log with colored output
- ✅ **Groot2 Integration**: Open trees in visual editor
- ✅ **Auto BT Server**: Automatically starts `bt_server_node`

## Installation

```bash
# Dependencies
sudo apt install ros-humble-rviz2

# Build
cd ~/ar_ws
colcon build --packages-select ar_ui
source install/setup.bash
```

## Usage

### Launch from RViz Panel

```bash
# Start MoveIt first
ros2 launch ar_control ar_moveit.launch.py desc:=<your_robot>

# Launch BT Manager from Panels menu
# In RViz: Panels → Add New Panel → ar_ui/BTManagerPanel
```

### Or Launch Standalone

```bash
# The panel starts when you click "Open Manager" in RViz
ros2 run ar_ui bt_manager  # If available as standalone
```

## Interface

```
┌─────────────────────────────────────────────────────────────────────┐
│ File   Tools   Help                                                 │
├──────────────────┬────────────────────┬─────────────────────────────┤
│ Projects         │ BehaviorTree XML   │ Execution                   │
│                  │                    │                             │
│ [project ▼] [↻]  │ <root ...>         │ ✓ Execution Completed       │
│                  │   <Sequence>       │                             │
│ Name: pick_place │     <MoveToJoint>  │ [▶ Run]  [■ Stop]           │
│ Motions: 8       │     ...            │ [🌳 Open in Groot2]         │
│ Waypoints: 5     │   </Sequence>      │                             │
│                  │ </root>            │ Execution Log:              │
│                  │                    │ ┌───────────────────────────┐
│                  │                    │ │ PLANNING: GoHome [0;0;0]  │
│                  │                    │ │ PLAN_SUCCESS: GoHome      │
│                  │                    │ │ EXECUTING: GoHome         │
│ [📄 Generate XML]│                    │ │ GoHome IDLE -> SUCCESS    │
│                  │                    │ └───────────────────────────┘
└──────────────────┴────────────────────┴─────────────────────────────┘
```

## Workflow

1. **Select Project** from dropdown (loads from `ar_projects/projects/`)
2. **Generate XML** - Converts YAML config to BehaviorTree XML
3. **Run** - Starts BT Server and executes the tree
4. **Monitor** - Watch live status in Execution Log

## Execution Log Colors

| Color | Status |
|-------|--------|
| 🟦 Cyan | `IDLE` state |
| 🟨 Yellow | `RUNNING` state |
| 🟩 Green | `SUCCESS` state |
| 🟥 Red | `FAILURE` state |
| 🟪 Magenta | Debug/Info |

## Menu Options

### File
- **Refresh Projects** - Reload project list
- **Exit** - Close the manager

### Tools
- **Clear Log** - Clear execution log
- **Open Projects Folder** - Browse project YAML files

### Help
- **About** - Version information

## ROS2 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/ar_bt/execute_command` | `std_msgs/String` | Publish | Send tree to execute |
| `/ar_bt/execution_status` | `std_msgs/String` | Subscribe | Receive status updates |

## Configuration

Projects are defined in `ar_projects/projects/<name>/config.yaml`:

```yaml
project:
  name: "my_motion"
  description: "Custom motion sequence"

defaults:
  planner: "pilz"
  velocity: 0.5

waypoints:
  home: [0, 0, 0, 0, 0, 0]
  target: [0.5, 0.3, -0.5, 0, 0, 0]

sequence:
  - type: move_joint
    waypoint: home
  - type: move_joint
    waypoint: target
```

## Code Flow

### Execution Flow

```
┌─────────────────────────────────────────────────────────────────────────┐
│ 1. User selects project from dropdown                                    │
│    └── onProjectSelected() → Load config.yaml → Show project info       │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│ 2. User clicks [Generate XML]                                            │
│    └── onGenerateXml()                                                   │
│        ├── ProjectManager::loadProject() → Parse YAML                    │
│        ├── XmlGenerator::generate() → Create BT XML                      │
│        └── Display XML in editor panel                                   │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│ 3. User clicks [▶ Run]                                                   │
│    └── onRunProject()                                                    │
│        ├── Save XML to /tmp/ar_project_<name>.xml                        │
│        ├── Start bt_server_node (if not running)                         │
│        ├── Publish filename to /ar_bt/execute_command                    │
│        └── Subscribe to /ar_bt/execution_status                          │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│ 4. Status updates received                                               │
│    └── statusCallback()                                                  │
│        ├── Parse status string (PLANNING/EXECUTING/SUCCESS/FAILURE)      │
│        ├── Update status_label_ (color + icon)                           │
│        └── appendLog() → Convert ANSI colors → Append to log_text_       │
└─────────────────────────────────────────────────────────────────────────┘
```

### Key Classes

| Class | File | Description |
|-------|------|-------------|
| `BTManagerWindow` | `bt_manager_window.cpp` | Main Qt window with all UI logic |
| `BTManagerPanel` | `bt_manager_panel.cpp` | RViz panel wrapper |
| `ProjectManager` | `ar_projects` | YAML loading & XML generation |

### File Structure

```
ar_ui/
├── include/ar_ui/
│   ├── bt_manager_window.h    # Main window class
│   └── bt_manager_panel.h     # RViz panel wrapper
├── src/
│   ├── bt_manager_window.cpp  # UI logic & ROS2 integration
│   └── bt_manager_panel.cpp   # Panel registration
└── CMakeLists.txt
```

## Troubleshooting

**"Not Connected"**
- Ensure ROS2 is sourced
- Check if MoveIt is running

**Projects not showing**
- Verify YAML files in `ar_projects/projects/`
- Click refresh button

**Execution fails**
- Check BT Server logs in terminal
- Verify joint values match robot DOF

## Quick kill for ROS processes

If a launch gets stuck, add this alias to your shell (e.g., `~/.bashrc`) to stop bringup/MoveIt/RViz in one shot:

```bash
alias ros_kill_all="pkill -f 'ar_control.*launch|move_group|rviz2|bt_manager|param_editor'"
```

Then run `source ~/.bashrc` (or start a new shell) and call `ros_kill_all`.
