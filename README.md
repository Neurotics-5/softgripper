# softgripper

Absolutely — here's a clear and professional **README section** for your gripper control package. You can copy-paste this into your GitHub README:

---

# 🦾 Soft Gripper Control Interface (ROS 2)

This package provides a modular ROS 2 interface for controlling a tendon-driven soft gripper using Dynamixel motors. It supports both **position control** and **force-assisted gripping**, and offers live feedback for status, force, and goal completion.

---

## 📦 Features

* Position control (`open`, `close`, arbitrary targets)
* Optional force control with PID-based correction
* Per-motor target speed profiles
* Goal state monitoring (`moving` / `reached`)
* Real-time feedback on joint position, current, and estimated force
* YAML-configurable motor IDs and motion limits

---

## 🚀 ROS 2 Commands (via `/gripper/command`)

| Command                                | Description                                                          |
| -------------------------------------- | -------------------------------------------------------------------- |
| `open`                                 | Opens the gripper to predefined open positions from the config file  |
| `close`                                | Closes the gripper to predefined closed positions                    |
| `move_to_position:<pos>`               | Moves all motors to the same position (e.g. `2.4` radians)           |
| `move_to_position:<id=pos,...>`        | Moves specific motors to target positions (e.g. `1=2.5,2=2.0`)       |
| `move_to_position:<id=pos@spd,...>`    | Sets per-motor position and speed (e.g. `1=2.5@50,2=2.0@30`)         |
| `move_to_position_force:<pos>:<force>` | Moves to a position and engages force control to maintain grip force |
| `set_operating_mode:position`          | Switches motors to position control mode                             |
| `set_operating_mode:force`             | Switches motors to current-based (force) control mode                |
| `set_torque_enable:on`                 | Enables motor torque                                                 |
| `set_torque_enable:off`                | Disables motor torque                                                |

---

## 📡 ROS 2 Feedback Topics

| Topic                   | Message Type      | Description                             |
| ----------------------- | ----------------- | --------------------------------------- |
| `/gripper/status`       | `std_msgs/String` | Live joint positions and motor currents |
| `/gripper/goal_reached` | `std_msgs/String` | `"moving"` or `"reached"`               |
| `/gripper/force_status` | `std_msgs/String` | Estimated total grip force (N)          |

---

## 🧾 Configuration

Edit the file:

```
config/gripper_config.yaml
```

You can define:

* Motor IDs
* Open/closed positions
* Min/max limits
* Pulley and joint radius
* Baudrate and port

---

## 🧪 Example Usage

### 1. Enable torque and set mode

```bash
ros2 topic pub --once /gripper/command std_msgs/String "data: 'set_operating_mode:position'"
ros2 topic pub --once /gripper/command std_msgs/String "data: 'set_torque_enable:on'"
```

### 2. Open or close

```bash
ros2 topic pub --once /gripper/command std_msgs/String "data: 'open'"
ros2 topic pub --once /gripper/command std_msgs/String "data: 'close'"
```

### 3. Move with speed profile

```bash
ros2 topic pub --once /gripper/command std_msgs/String "data: 'move_to_position:1=3.0@40,2=2.5@30'"
```

### 4. Monitor feedback

```bash
ros2 topic echo /gripper/status
ros2 topic echo /gripper/goal_reached
ros2 topic echo /gripper/force_status
```

---

Let me know if you’d like this exported as a markdown `.md` file or formatted with a collapsible GitHub TOC.
