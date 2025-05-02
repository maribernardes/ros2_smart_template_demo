# 3DOF SmartTemplate demo package: ros2_smart_template_demo

## Quick demo test
To give it a try on simulation mode, launch:
```
ros2 launch smart_template_demo robot.launch.py gui:=true rviz:=true
```

## Overview
This repository contains:
- ROS2 package 'smart_template_demo' for emulated virtual 3DOF SmartTemplate
- ROS2 package 'robot_description' for publishing the SmartTemplate description from URDF

Currently tested for:
- ROS2 Humble, Ubuntu 22.04 / ROS2 Jazzy, Ubuntu 22.04

## Description
### Subscribers
- '/desired_position', a Point with the desired end-effector position
- '/desired_command', a String with one of the predefined command ('HOME', 'RETRACT', 'ABORT')

### Publishers
- '/stage/state/guide_pose', a PoseStamped with the current position of the template. It's refresh rate is given by the "timer_period"
- '/joint_states', a JointState with the current joint values of the template. It's refresh rate is given by the "timer_period"

### Launch file
- robot.launch.py
  * Argument: "rviz"
    * false - NO rviz
    * true - loads rviz
  * Argument: "gui"
    * false - NO rqt GUI custom plugin
    * true - loads rqt GUI custom plugin

## Usage <a name="usage"></a>
## Axis
- Left-right (horizontal): x
- Inferior-superior (insertion): y
- Anterior-posterior (vertical): z