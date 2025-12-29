# AR BehaviorTree Integration

## Overview

BehaviorTree integration for AR robot motion sequencing using BehaviorTree.CPP v3. Enables visual programming of robot tasks with drag-and-drop tree editing via Groot2.

## Features

- ✅ **Action Nodes**: `MoveToJoint`, `PlanToJoint`, `SetPlanner`, `SetBlendRadius`, `SetVelocityScaling`
- ✅ **Condition Nodes**: `IsAtJoint`, `IsPlanValid`, `IsRobotReady`
- ✅ **Modular SubTrees**: Primitives (`go_home`, `setup_motion`) and Skills (`pick_object`, `place_object`)
- ✅ **Fallback Recovery**: Built-in error handling patterns
- ✅ **BT Server**: Background service for UI integration
- ✅ **Groot2 Support**: Visual editor with custom node palette

## Installation

```bash
# Install BehaviorTree.CPP
sudo apt install ros-humble-behaviortree-cpp-v3

# Build
cd ~/ar_ws
colcon build --packages-select ar_bt
source install/setup.bash
```

## Quick Start

### 1. Launch Robot & MoveIt
```bash
ros2 launch ar_control ar_moveit.launch.py desc:=<your_robot>
```

### 2. Run BT Executor (one-shot)
```bash
ros2 run ar_bt bt_executor_node --ros-args -p tree_file:=pick_place.xml
```

### 3. Run BT Server (for UI)
```bash
ros2 run ar_bt bt_server_node --ros-args -p planning_group:=Arm
```

## Available Nodes

### Action Nodes
| Node | Input Ports | Description |
|------|-------------|-------------|
| `MoveToJoint` | `target_joints` | Plan & execute to joint target |
| `PlanToJoint` | `target_joints` | Plan only (no execution) |
| `SetPlanner` | `pipeline`, `planner` | Configure motion planner |
| `SetBlendRadius` | `radius` | Set PILZ blend radius |
| `SetVelocityScaling` | `factor` | Set velocity (0.0-1.0) |
| `SetAccelerationScaling` | `factor` | Set acceleration (0.0-1.0) |

### Condition Nodes
| Node | Input Ports | Description |
|------|-------------|-------------|
| `IsAtJoint` | `target_joints`, `tolerance` | Check if robot at position |
| `IsPlanValid` | `plan_key` | Check if plan exists in blackboard |
| `IsRobotReady` | - | Check if MoveIt is operational |

### Built-in Nodes (BT.CPP)
- `AlwaysSuccess`, `AlwaysFailure`, `Sequence`, `Fallback`, `Retry`, `Delay`

## Tree Structure

```
trees/
├── primitives/           # Atomic actions
│   ├── go_home.xml       # Move to home position
│   └── setup_motion.xml  # Configure planner
├── skills/               # Composed behaviors  
│   ├── pick_object.xml   # Pick with Fallback recovery
│   └── place_object.xml  # Place with Fallback recovery
├── pick_place.xml        # Complete pick-place mission
└── simple_motion_test.xml
```

## Creating Trees

### Method 1: XML
```xml
<root BTCPP_format="4">
  <BehaviorTree ID="MyTree">
    <Sequence>
      <SetPlanner pipeline="pilz" planner="PTP"/>
      <SetVelocityScaling factor="0.5"/>
      <MoveToJoint target_joints="0;0;0;0;0;0" name="Home"/>
      <MoveToJoint target_joints="0.5;0.3;-0.5;0;0;0" name="Target"/>
    </Sequence>
  </BehaviorTree>
</root>
```

### Method 2: Groot2 (Visual Editor)
1. Launch: `groot2`
2. Load palette: **File → Load Palette → `config/bt_plugins.xml`**
3. Drag & drop nodes to build tree
4. Save as XML → Run with executor

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/ar_bt/execute_command` | `std_msgs/String` | Send tree file to execute |
| `/ar_bt/execution_status` | `std_msgs/String` | Execution status updates |

## Architecture & Code Flow

### System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           ar_ui (Qt Panel)                               │
│  ┌─────────────┐  ┌─────────────┐  ┌──────────────────────────────────┐ │
│  │ Project List│  │ XML Editor  │  │ Execution Log                    │ │
│  │ (YAML→XML)  │  │ (Generated) │  │ PLANNING: GoHome [0;0;0;0;0;0]   │ │
│  └─────────────┘  └─────────────┘  │ PLAN_SUCCESS: GoHome             │ │
│         │                          │ GoHome IDLE -> SUCCESS           │ │
│         ▼                          └──────────────────────────────────┘ │
│  [▶ Run] → Publish to /ar_bt/execute_command                            │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                        ar_bt (bt_server_node)                            │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ BehaviorTreeFactory                                                 │ │
│  │  ├── registerARBTNodes()      → Action Nodes                        │ │
│  │  └── registerARBTConditionNodes() → Condition Nodes                 │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                    │                                     │
│                                    ▼                                     │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ BT::Tree (loaded from XML)                                          │ │
│  │  └── tree.tickRoot() → Execute nodes in Sequence/Fallback          │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                    │                                     │
│  Publish status to /ar_bt/execution_status                              │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                    AR BT Action Nodes                                    │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                   │
│  │ MoveToJoint  │  │ SetPlanner   │  │ IsAtJoint    │                   │
│  │ (Stateful)   │  │ (Sync)       │  │ (Condition)  │                   │
│  └──────┬───────┘  └──────────────┘  └──────────────┘                   │
│         │                                                                │
│         ▼                                                                │
│  publishStatus("PLANNING: GoHome [0.00;0.00;...]")                      │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                     ar_planning_interface                                │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ ArPlanningInterface                                                 │ │
│  │  ├── setTargetJoints()                                              │ │
│  │  ├── plan() → MoveGroupInterface::Plan                              │ │
│  │  ├── execute() → MoveGroupInterface::execute                        │ │
│  │  └── visualizeTrajectory() → RViz path display                      │ │
│  └────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                           MoveIt 2                                       │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                      │
│  │ OMPL/PILZ   │  │ Trajectory  │  │ Controller  │                      │
│  │ Planner     │→ │ Execution   │→ │ Interface   │→ Robot Hardware      │
│  └─────────────┘  └─────────────┘  └─────────────┘                      │
└─────────────────────────────────────────────────────────────────────────┘
```

### Code Flow: MoveToJoint Execution

```
1. User clicks [▶ Run] in ar_ui
           │
           ▼
2. BTManagerWindow::onRunProject()
   └── Publish tree filename to /ar_bt/execute_command
           │
           ▼
3. bt_server_node receives command
   ├── Load XML: factory.createTreeFromFile(path)
   ├── Create logger: BT::StdCoutLogger(tree)
   └── Execute: tree.tickRoot()
           │
           ▼
4. MoveToJoint::onStart() called
   ├── getInput<vector<double>>("target_joints")
   ├── publishStatus("PLANNING: GoHome [0.00;0.00;...]")
   ├── planning_interface_->setTargetJoints(target)
   ├── planning_interface_->plan(plan)
   │   └── Returns SUCCESS/FAILURE
   ├── publishStatus("PLAN_SUCCESS: GoHome")
   ├── planning_interface_->execute(plan)
   │   └── Blocks until motion complete
   └── publishStatus("EXECUTE_SUCCESS: GoHome")
           │
           ▼
5. ar_ui receives status via /ar_bt/execution_status
   └── appendLog() with ANSI→HTML color conversion
```

### File Structure

```
ar_bt/
├── include/ar_bt/
│   ├── bt_action_nodes.h      # Action node class definitions
│   └── bt_condition_nodes.h   # Condition node definitions
├── src/
│   ├── bt_action_nodes.cpp    # Node implementations + registration
│   ├── bt_condition_nodes.cpp # Condition implementations
│   ├── bt_executor_node.cpp   # Standalone executor (one-shot)
│   └── bt_server_node.cpp     # Background server (for UI)
├── trees/
│   ├── primitives/            # Atomic subtrees
│   ├── skills/                # Composed subtrees
│   └── *.xml                  # Mission trees
├── config/
│   └── bt_plugins.xml         # Groot2 node palette
└── launch/
    └── bt_executor.launch.py
```

## Troubleshooting

**"behaviortree_cpp not found"**
```bash
sudo apt install ros-humble-behaviortree-cpp-v3
```

**Execution fails**
- Ensure MoveIt is running
- Check joint count matches your robot
- Verify `planning_group` parameter

## Resources

- [BehaviorTree.CPP](https://www.behaviortree.dev/)
- [Groot2 Editor](https://www.behaviortree.dev/groot)
