# 3DOF SmartTemplate demo package:

## Quick demo test
To give it a try on simulation mode, launch:
```
ros2 launch smart_template_demo robot.launch.py gui:=true rviz:=true
```

## Overview
This repository contains:
- `smart_template_demo` for emulated virtual 3DOF SmartTemplate
- `smart_template_description` for SmartTemplate URDF description files

Currently tested for:
- ROS2 Humble, Ubuntu 22.04 / ROS2 Jazzy, Ubuntu 22.04

## Description
### Robot 
```
world
├── base_joint (fixed)
    └── base_link
        └── vertical_joint (Z-axis)
            └── vertical_link
                └── horizontal_joint (X-axis)
                    └── horizontal_link
                        └── insertion_joint (Y-axis)
                            └── needle_link (end-effector)
```

| Joint Name     | Type       | Axis       | Parent Link   | Child Link    | Range (m)    |
|----------------|------------|------------|---------------|--------------|-------------|
| vertical_joint | prismatic | Z (0,0,1)  | base_link     | vertical_link | ±0.025      |
| horizontal_joint | prismatic | X (1,0,0)  | vertical_link | horizontal_link | ±0.03       |
| insertion_joint | prismatic | Y (0,1,0)  | horizontal_link | needle_link   | 0.000 → 0.115 |

### Published Topics

| Topic             | Type                       | Description                                                                         |
|-------------------|----------------------------|-------------------------------------------------------------------------------------|
| `/end_effector_pose` | `geometry_msgs/PoseStamped` | Current needle tip pose in the world frame, computed via TF transform from `needle_link` |
| `/joint_states`   | `sensor_msgs/JointState`   | Current joint values (3 prismatic joints) in meters, as expected by standard ROS tools |

### Subscribed Topics

| Topic             | Type                       | Description                                                                          |
|-------------------|----------------------------|--------------------------------------------------------------------------------------|
| `/desired_position` | `geometry_msgs/PoseStamped` | Target pose to align the robot tip to, typically in the world or scanner frame         |
| `/desired_command` | `std_msgs/String`          | Predefined motion commands: `HOME`, `RETRACT`, or `ABORT`                             |

### Launch file
- robot.launch.py
  * Argument: `rviz`
    * `false` - NO rviz
    * `true` - loads rviz
  * Argument: `gui`
    * `false` - NO rqt GUI custom plugin
    * `true` - loads rqt GUI custom plugin
