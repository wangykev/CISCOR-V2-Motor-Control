# CISCOR-V2 Motor Control (ROS 2)

ROS 2 nodes and helpers for controlling **CubeMars AK-series motors (V2)** over **SocketCAN** in **MIT mode** on Linux/Raspberry Pi.

This package provides per-motor ROS 2 nodes, simple topics for commands and telemetry, safe limit clamping per motor, and optional multi-motor setups via namespaces.

---

## Features

- `motor_node` (one per motor), easy to run multiple motors with namespaces
- Command topic `/<ns>/mit_cmd` (`Float64MultiArray = [pos, vel, Kp, Kd, torque]`)
- Telemetry on `/<ns>/state_line` (human-readable) and `/<ns>/joint_states` (standard)
- Utility control via `/<ns>/special` (`start`, `zero`, `clear`, `exit`)
- Per-motor safety limits (P/V/T/Kp/Kd) with clamping

---

## Requirements

- ROS 2 (Humble or Jazzy) and `colcon`
- Python 3.10+
- `can-utils` and a working SocketCAN interface (e.g., `can0`)
- A CAN adapter (MCP2515 HAT or USB-CAN)

---

## Installation

1) Clone into your ROS 2 workspace:

```bash
# Example workspace path
cd ~/v2_ws/src
git clone https://github.com/wangykev/CISCOR-V2-Motor-Control.git .
cd ..
colcon build --packages-select cubemars_v2_ros --symlink-install
source install/setup.bash
```

## Bring up CAN (adjust bitrate for your hardware):

```bash
sudo ip link set can0 up type can bitrate 1000000
```


## Running a single motor:

```bash 
ros2 run cubemars_v2_ros motor_node --ros-args \
  -p can_interface:=can0 \
  -p can_id:=3 \
  -p motor_type:=AK70-10 \
  -p auto_start:=true
```

# Sending MIT command:
```bash
# Float64MultiArray: [pos, vel, Kp, Kd, torque]
ros2 topic pub /mit_cmd std_msgs/Float64MultiArray "{data: [0.0, 6.28, 0.0, 1.0, 0.0]}"
```


# Monitor: 
```bash
ros2 topic echo /state_line
```


# Special Commands:
```bash
ros2 topic pub --once /special std_msgs/String "data: start"  # motor automatically starts when you open the node, if you call exit, you must call start before you can control the motor again. You can set the values before you give it the start command. Once started the motor should emit a small hum. 
ros2 topic pub --once /special std_msgs/String "data: zero" #sets the current position to the 0 position
ros2 topic pub --once /special std_msgs/String "data: exit" #disconnects the motor from recieving can commands
ros2 topic pub --once /special std_msgs/String "data: clear" #sets all the values on the motor to 0
```


## Running multiple Motors (example)

# Motor A (ID 3) in one terminal
```bash
cd ~/v2_ws
colcon build --packages-select cubemars_v2_ros --symlink-install
source install/setup.bash

ros2 run cubemars_v2_ros motor_node \
  --ros-args -r __ns:=/ak70 -r __node:=motor_node_ak70 \
  -p can_interface:=can0 -p can_id:=3 -p motor_type:=AK70-10 -p joint_name:=ak70

```


# Motor B (ID 4) in another terminal 
```Bash
source ~/v2_ws/install/setup.bash

ros2 run cubemars_v2_ros motor_node \
  --ros-args -r __ns:=/ak80 -r __node:=motor_node_ak80 \
  -p can_interface:=can0 -p can_id:=4 -p motor_type:=AK80-64 -p joint_name:=ak80

```
# Commands:
```bash
ros2 topic echo /ak70/state_line
ros2 topic echo /ak80/state_line


ros2 topic pub --once /ak70/special std_msgs/String "data: start"
ros2 topic pub --once /ak80/special std_msgs/String "data: start"

ros2 topic pub --once /ak70/special std_msgs/String "data: zero"
ros2 topic pub --once /ak80/special std_msgs/String "data: zero"

ros2 topic pub --once /ak70/special std_msgs/String "data: exit"
ros2 topic pub --once /ak80/special std_msgs/String "data: exit"

ros2 topic pub --once /ak70/special std_msgs/String "data: clear"
ros2 topic pub --once /ak80/special std_msgs/String "data: clear"


# AK70: velocity-only (v=6.28 rad/s, Kd=1.0)
ros2 topic pub /ak70/mit_cmd std_msgs/Float64MultiArray "{data: [0.0, 6.28, 0.0, 1.0, 0.0]}"
ros2 topic pub /ak70/mit_cmd std_msgs/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0]}"

# AK80: example
ros2 topic pub /ak80/mit_cmd std_msgs/Float64MultiArray "{data: [0.0, 1.0, 0.0, 0.8, 0.0]}"
ros2 topic pub /ak80/mit_cmd std_msgs/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0]}"
```

# General Pattern 
General pattern (N motors)

Use a distinct __ns and __node for each instance.

Set each motor’s can_id and motor_type correctly.

Publish to /<ns>/mit_cmd and /<ns>/special for that motor.

Echo /<ns>/state_line (and /<ns>/joint_state, etc.) to monitor.


