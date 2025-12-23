# AR BehaviorTree Integration

## Overview
BehaviorTree integration for AR robot motion sequencing using BehaviorTree.CPP v4. Enables visual programming of robot tasks with drag-and-drop tree editing.

## Installation

### 1. Install BehaviorTree.CPP
```bash
sudo apt install ros-humble-behaviortree-cpp-v3
```

### 2. Install Groot2 (Optional - Visual Editor)
```bash
sudo apt install ros-humble-groot
# OR download from: https://www.behaviortree.dev/groot
```

### 3. Build ar_bt
```bash
cd ~/robot6dof
colcon build --packages-select ar_bt
source install/setup.bash
```

## Quick Start

### 1. Launch MoveIt
```bash
ros2 launch ar_control ar_moveit.launch.py desc:=<your_robot>
```

### 2. Run Simple Test
```bash
ros2 launch ar_bt bt_executor.launch.py tree_file:=simple_motion_test.xml
```

### 3. Run Pick-Place Example
```bash
ros2 launch ar_bt bt_executor.launch.py tree_file:=pick_place.xml
```

## Available BT Nodes

### Motion Nodes
- **MoveToJoint** - Plan and execute to joint target
  - Input: `target_joints` (semicolon-separated, e.g., "0;0.5;-0.5;0;0;0")
  
- **PlanToJoint** - Plan to joint target (no execution)
  - Input: `target_joints`
  - Output: `success` (bool)

- **SetPlanner** - Configure planner
  - Input: `pipeline` ("ompl" or "pilz")
  - Input: `planner` ("RRTConnect", "PTP", etc.)

- **SetBlendRadius** - Set PILZ blend radius
  - Input: `radius` (meters)

## Creating Your Own Trees

### Method 1: XML (Manual)
Create XML file in `trees/` directory:
```xml
<root BTCPP_format="4">
  <BehaviorTree ID="MySequence">
    <Sequence>
      <SetPlanner pipeline="pilz" planner="PTP"/>
      <MoveToJoint target_joints="0;0;0;0;0;0"/>
      <MoveToJoint target_joints="0;0.5;-0.5;0;0;0"/>
    </Sequence>
  </BehaviorTree>
</root>
```

### Method 2: Groot2 (Visual)
1. Launch Groot2: `groot2`
2. Load `config/bt_plugins.xml` to see AR nodes
3. Drag and drop nodes to create tree
4. Save as XML
5. Run with executor

## Example Trees Included

### simple_motion_test.xml
Basic test sequence: Home → Position1 → Home

### pick_place.xml
Pick-and-place demo with PILZ blending

## Architecture

```
BT Executor
    ↓
BT Action Nodes (MoveToJoint, SetPlanner, etc.)
    ↓
ar_planning_interface
    ↓
MoveIt 2
```

## Troubleshooting

**Build fails with "behaviortree_cpp not found":**
- Install: `sudo apt install ros-humble-behaviortree-cpp-v3`
- Or build from source (see installation steps)

**Tree execution fails:**
- Ensure MoveIt is running first
- Check joint limits in your tree matches robot DOF
- Verify planning group name parameter

## Next Steps

1. **Add Condition Nodes**: IsAtJoint, IsAtPose, PlannerReady
2. **Add Scene Nodes**: AddCollisionBox, RemoveObject (using ar_scene)
3. **Add Gripper Nodes**: Open, Close (integrate with your gripper)
4. **Create Custom Trees**: For your specific tasks (welding, assembly, etc.)

## Resources

- [BehaviorTree.CPP Docs](https://www.behaviortree.dev/)
- [Groot2 Editor](https://www.behaviortree.dev/groot)
- [Example Trees](https://www.behaviortree.dev/docs/learn-the-basics/bt_basics)
