# RMF VDA5050 Adapter — Setup & Run Guide

Complete instructions for replicating this environment from a fresh ROS 2 Humble installation.

---

## Prerequisites

- Ubuntu 22.04 (Jammy)
- ROS 2 Humble installed to `/opt/ros/humble`
  (follow the [official install guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) if not done yet)

---

## 1. Install System Dependencies

```bash
sudo apt update

# Build tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

# MQTT broker
sudo apt install -y mosquitto mosquitto-clients

# nlohmann JSON (C++ header-only library)
sudo apt install -y nlohmann-json3-dev

# Python MQTT client (used by the mock robot)
sudo apt install -y python3-paho-mqtt
```

---

## 2. Install Open-RMF Binary Packages

```bash
sudo apt install -y \
  ros-humble-rmf-fleet-adapter \
  ros-humble-rmf-traffic \
  ros-humble-rmf-traffic-ros2 \
  ros-humble-rmf-task-ros2 \
  ros-humble-rmf-building-map-tools \
  ros-humble-rmf-visualization-navgraphs \
  ros-humble-rmf-visualization-fleet-states \
  ros-humble-rviz2
```

---

## 3. Create the Workspace and Clone the Repo

```bash
mkdir -p ~/rmf_ws/src
cd ~/rmf_ws/src
git clone https://github.com/jn799/ros2_openrmf_vda5050_adapter.git .
```

> Note: the repo must be public (or you must be authenticated) for the clone to succeed.

---

## 4. Initialise rosdep and Install Remaining Dependencies

```bash
source /opt/ros/humble/setup.bash

cd ~/rmf_ws
rosdep init          # skip if already done system-wide
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## 5. Build

```bash
cd ~/rmf_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rmf_vda5050_adapter vda5050_mock_robot
source install/setup.bash
```

Add the source commands to your shell profile so you don't need to repeat them in every terminal:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/rmf_ws/install/setup.bash" >> ~/.bashrc
```

---

## 6. Start the MQTT Broker

Installing mosquitto via `apt` enables and starts it as a systemd service automatically. Check if it's already running:

```bash
systemctl is-active mosquitto
```

If it shows `active`, you're done — no need to start it manually. If not:

```bash
sudo systemctl start mosquitto
```

Mosquitto listens on `localhost:1883` by default. To have it start automatically on every boot:

```bash
sudo systemctl enable mosquitto
```

---

## 7. Launch the Full RMF Stack

In a new terminal (with both setup files sourced):

```bash
source /opt/ros/humble/setup.bash && source ~/rmf_ws/install/setup.bash

ros2 launch rmf_vda5050_adapter rmf_stack.launch.py headless:=false
```

This starts (in dependency order):

| Component | Role |
|---|---|
| `rmf_traffic_schedule` | Central traffic brain |
| `building_map_server` | Publishes the warehouse floor plan |
| `rmf_task_dispatcher` | Manages task bidding |
| `rmf_vda5050_adapter` | Fleet adapter (starts 2 s after schedule) |
| `navgraph_visualizer_node` | Renders waypoints/lanes in RViz |
| `fleetstates_visualizer_node` | Renders robot positions in RViz |
| `rviz2` | Pre-configured RMF visualization |

---

## 8. Start the Mock Robot

In a separate terminal:

```bash
source /opt/ros/humble/setup.bash && source ~/rmf_ws/install/setup.bash

ros2 run vda5050_mock_robot mock_robot
```

The mock robot connects to the MQTT broker, receives VDA5050 orders from the adapter, and simulates motion between waypoints.

---

## 9. Dispatch a Patrol Task

In another terminal:

```bash
source /opt/ros/humble/setup.bash && source ~/rmf_ws/install/setup.bash

ros2 run rmf_vda5050_adapter dispatch_patrol <waypoint> [waypoint2 waypoint3 ...]
```

### Available waypoints

| Name | Coordinates |
|---|---|
| `start` | (0, 0) |
| `wp1` | (5, 0) |
| `wp2` | (10, 0) |
| `wp3` | (5, 5) |
| `charging` | (5, 10) |

### Map topology

```
start(0,0) — wp1(5,0) — wp2(10,0)
                 |
               wp3(5,5)
                 |
            charging(5,10)
```

### Examples

```bash
# Single destination
ros2 run rmf_vda5050_adapter dispatch_patrol wp2

# Multi-stop patrol
ros2 run rmf_vda5050_adapter dispatch_patrol wp1 wp3 charging wp1 start
```

---

## Terminal Layout Summary

| Terminal | Command |
|---|---|
| 1 | `ros2 launch rmf_vda5050_adapter rmf_stack.launch.py headless:=false` |
| 2 | `ros2 run vda5050_mock_robot mock_robot` |
| 3 | `ros2 run rmf_vda5050_adapter dispatch_patrol <waypoints...>` |

> Mosquitto runs as a system service and does not need a dedicated terminal.
