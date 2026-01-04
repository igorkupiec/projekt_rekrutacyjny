# Joy to Twist

ROS 2 package for robot teleoperation via gamepad.

## Overview
Converts gamepad axes into `TwistStamped` velocity commands.

## Controls
* **Left Stick:** Linear velocity (Forward/Backward)
* **Right Stick:** Angular velocity (Clockwise/Counter-clockwise)

---

## Nodes

### `joy_to_twist_node`
The core translator node.
* **Subscribes:** `/joy`
* **Publishes:** `/robot_velocity`, `/input_pada`
* **Parameters:**
    * `joy_topic` (string, default: "joy")
    * `input_topic` (string, default: "input_pada")
    * `vel_topic` (string, default: "robot_velocity")

### `joy_to_twist_sub`
Console-based diagnostic tool.
* **Subscribes:** `/robot_velocity`, `/input_pada`
* **Parameters:**
    * `input_topic` (string, default: "input_pada")
    * `vel_topic` (string, default: "robot_velocity")

### `joy_to_twist_tui`
Terminal-based visualizer.
* **Subscribes:** `/input_pada`

---

## Topics
* **`/joy`** (`sensor_msgs/msg/Joy`): Raw data stream from the gamepad driver.
* **`/input_pada`** (`geometry_msgs/msg/TwistStamped`): Translated data representing stick positions.
* **`/robot_velocity`** (`geometry_msgs/msg/TwistStamped`): Final linear and angular velocity commands for the robot.

---

## Quick Start

1. **Launch Joy Driver:**
   ```bash
   ros2 run joy joy_node