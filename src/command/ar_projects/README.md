# AR Projects - Motion Sequence Generator

YAML-configurable motion sequence generator for BehaviorTree XML files, with Groot2 integration.

## Quick Start

### From RViz
1. Launch RViz: `ros2 launch ar_control ar_rviz.launch.py desc:=<robot>`
2. The **BehaviorTree** panel appears in the sidebar
3. Select a project from the dropdown
4. Click **🌳 Open in Groot** to edit/view the tree
5. Click **▶ Run Project** to execute

### Install Groot2 (Optional but Recommended)
```bash
# Download Groot2 AppImage
wget https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.6.1-linux-installer.run
chmod +x Groot2-v1.6.1-linux-installer.run
./Groot2-v1.6.1-linux-installer.run
```

Or download from: https://www.behaviortree.dev/groot

---

## Project Structure

```
ar_projects/
├── config/
│   └── defaults.yaml        # Global defaults (all projects)
└── projects/
    ├── pick_place/
    │   └── config.yaml      # Project configuration
    ├── point_circle/
    │   └── config.yaml
    └── demo_sequence/
        └── config.yaml
```

## Creating a New Project

1. Create a folder: `projects/my_project/`
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

---

## Available Motion Types

| Type | Description | Parameters |
|------|-------------|------------|
| `move_joint` | Move to joint config | `joints: [...]` or `waypoint: "name"` |
| `delay` | Wait for duration | `duration_ms: 500` |
| `set_planner` | Configure planner | `pipeline`, `planner_id` |
| `set_velocity` | Set velocity scaling | `factor: 0.5` |
| `set_acceleration` | Set acceleration | `factor: 0.5` |

---

## ar_ui Panel Features

The RViz panel provides:
- **📄 Generate XML** - Convert YAML to BehaviorTree XML
- **🌳 Open in Groot** - Launch Groot2 to edit/visualize the tree
- **▶ Run Project** - Generate XML and execute immediately
- **■ Stop** - Stop execution
- **Execution Log** - View execution status messages

---

## Command Line Usage

```bash
# Generate XML from project
ros2 run ar_projects project_runner --ros-args -p project:=pick_place

# List available projects
ros2 run ar_projects project_runner --ros-args -p list_projects:=true
```
