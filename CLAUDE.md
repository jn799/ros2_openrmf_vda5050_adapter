# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
cd ~/rmf_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rmf_vda5050_adapter vda5050_mock_robot
source install/setup.bash
```

Single package rebuild after a code change:
```bash
colcon build --packages-select rmf_vda5050_adapter
```

No automated tests — verified by running the full stack and dispatching patrol tasks.

## Running the Stack

Source both setup files in every terminal before running any `ros2` command:
```bash
source /opt/ros/humble/setup.bash && source ~/rmf_ws/install/setup.bash
```

**Option A — combined launch file (recommended):**
```bash
ros2 launch rmf_vda5050_adapter rmf_stack.launch.py
ros2 run vda5050_mock_robot mock_robot
```

Optional launch args: `headless:=true` (skip rviz2), `map_name:=L1`

**Option B — individual nodes (start in this order):**
```bash
mosquitto                                          # MQTT broker (port 1883)
ros2 run rmf_traffic_ros2 rmf_traffic_schedule
ros2 run rmf_building_map_tools building_map_server ~/rmf_ws/install/rmf_vda5050_adapter/share/rmf_vda5050_adapter/config/warehouse.building.yaml
ros2 run rmf_task_ros2 rmf_task_dispatcher
ros2 run rmf_vda5050_adapter adapter               # wait ~2s after schedule
ros2 run rmf_visualization_navgraphs navgraph_visualizer_node
ros2 run rmf_visualization_fleet_states fleetstates_visualizer_node --ros-args -p vda5050_fleet_radius:=0.3
ros2 run rviz2 rviz2 -d ~/rmf_ws/install/rmf_vda5050_adapter/share/rmf_vda5050_adapter/config/rmf.rviz
ros2 run vda5050_mock_robot mock_robot
```

The adapter accepts an optional config path as a CLI argument (falls back to the package default `config/adapter.yaml`):
```bash
ros2 run rmf_vda5050_adapter adapter /path/to/custom_adapter.yaml
```

## Dispatching Tasks

```bash
ros2 run rmf_vda5050_adapter dispatch_patrol <waypoint> [waypoint2 ...]
```

Available waypoints (defined in `src/rmf_vda5050_adapter/config/nav_graph.yaml`): `start`, `wp1`, `wp2`, `wp3`, `charging`

```
start(0,0) — wp1(5,0) — wp2(10,0)
                 |
               wp3(5,5)
                 |
            charging(5,10)
```

## Architecture

Two ROS2 packages under `src/`:

### `rmf_vda5050_adapter/` — C++ fleet adapter (Open-RMF ↔ VDA5050 over MQTT)

- `src/VDA5050Adapter.cpp` — Top-level node: MQTT client, robot discovery, state parsing, RMF fleet setup. Calls `set_task_planner_params()` (required for task bidding) and `consider_patrol_requests()`.
- `src/RobotHandle.cpp` — Per-robot command handler implementing `RobotCommandHandle`. Key methods:
  - `follow_new_path()` — receives RMF waypoints, converts to VDA5050 order, sends via MQTT
  - `on_state_update()` — called on each MQTT state message; updates RMF position and calls `_arrival_est_cb()`
  - `send_order()` / `send_instant_action()` — VDA5050 JSON serialization
- `src/main.cpp` — Loads `adapter.yaml` (or CLI-provided path), constructs `VDA5050Adapter`, runs until SIGINT/SIGTERM.
- `scripts/dispatch_patrol.py` — Publishes patrol task requests to `/task_api_requests` with `QoS=TRANSIENT_LOCAL`
- `config/adapter.yaml` — Fleet name, MQTT host/port, VDA5050 version, robot kinematics, battery params
- `config/nav_graph.yaml` — Waypoint graph (edit to match real facility)

### `vda5050_mock_robot/` — Python simulated AGV

- `vda5050_mock_robot/mock_robot.py` — Connects to MQTT, receives VDA5050 orders, drives node-by-node via linear interpolation (0.1s timestep), publishes state at 1 Hz. Motion runs in a background thread.

## Key Design Notes

- **MQTT topic scheme**: `uagv/{vda5050_version}/{manufacturer}/{serial}/{kind}`. Adapter subscribes to `uagv/v2/+/+/state` and `uagv/v2/+/+/connection`. The `version` field comes from `adapter.yaml` (`vda5050.version`, default `"v2"`).
- **Arrival estimator callback** (`_arrival_est_cb`) must be called in `on_state_update()` with ETA = `dist / max_speed`. Without it, RMF triggers a replan every ~10 seconds.
- **Task bidding** requires `set_task_planner_params()` to be called during fleet setup — omitting it silently prevents the fleet from bidding.
- **`dispatch_patrol.py` QoS** must use `TRANSIENT_LOCAL` durability to match `rmf_task_dispatcher`.
- **Node IDs** in VDA5050 orders use prefix `g{index}` for graph waypoints (e.g. `g0`, `g4`) and `path_{i}` for free-space positions.
- Robot is registered at waypoint index 0 (`start`) on first MQTT state message. Idle position is reported by waypoint index when `last_node_id` matches a `g{n}` node.
