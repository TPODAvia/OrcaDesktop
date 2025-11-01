# RMF Manager

A lightweight, ROS 2-based task manager and web dashboard for mobile robots. It includes:

* `rmf_manager_core` — a single-node task scheduler + Nav2 client with simple graph-based routing.
* `rmf_manager_dashboard` — a tiny web UI (task queue + SVG map visualizer).
* `rmf_manager_msgs` — message and service definitions used by the core and UI.

The goal is to give you a practical, minimal setup to queue “patrol / delivery / go-to / return home” tasks, visualize the map + robot pose, and integrate with existing Nav2 & docking stacks.

---

## Features

* Queue, reorder, and cancel tasks (patrol / delivery / go-to / return).
* Graph loader from a building YAML (vertices/lanes) + pixel↔meter transform.
* Nav2 `NavigateToPose` actions with optional “handoff radius” for smoother progress.
* Optional docking/undocking via OpenNav Docking actions.
* Simple collision “preskip” heuristic using a local costmap (skip/expand target radius).
* Web UI

  * **Task Manager** (Jinja/Flask): schedule, reorder, cancel tasks.
  * **Visualizer** (SVG): background image, graph overlay, live pose & planned path.
* Domain Bridge config included for multi-domain setups.

---

## Package Layout

```
rmf_manager/
├── rmf_manager_core/         # C++ node + launch + config (incl. domain_bridge.yaml)
├── rmf_manager_dashboard/    # Python node + web assets (templates/static) + launch
└── rmf_manager_msgs/         # .msg / .srv definitions
```

---

## Requirements

* Ubuntu 22.04 + ROS 2 Humble (rclcpp, rclpy, nav2, tf2, domain\_bridge)
* Python deps (dashboard):

  * `flask`, `waitress`, `pyyaml` (`pip install --user flask waitress pyyaml`)
* Nav2 bringup running (for navigation) and, optionally, OpenNav docking servers.

---

## Build

```bash
# From your colcon workspace root
colcon build --symlink-install
source install/setup.bash
```

---

## Quickstart (single machine)

1. **Prepare config files** (placed by default under `rmf_manager_core/share/rmf_manager_core/config/`):

   **`floor_params.yaml` (pixel→map transform):**

   ```yaml
   /**:
     ros__parameters:
       rotation: 0.0            # radians, CCW
       scale: 0.05              # meters per pixel
       translation_x: 0.0       # meters
       translation_y: 0.0       # meters
   ```

   **`building.yaml` (graph + background image):**

   ```yaml
   levels:
     L1:
       drawing:
         filename: map.png
       vertices:
         # [x_px, y_px, name?]
         - [100, 200, "station"]
         - [450, 220, "a"]
         - [820, 360, "b"]
       lanes:
         # [idx_from, idx_to, {attrs}]
         - [0, 1]
         - [1, 2, { bidirectional: true, speed_limit: 0.6 }]
   ```

   Place `map.png` in the same folder as `building.yaml`, or adjust the path accordingly.

2. **Run the core:**

   ```bash
   ros2 launch rmf_manager_core rmf_manager.launch.py
   ```

   * This launches:

     * `rmf_manager_node` with parameters (see below).
     * `domain_bridge` using `config/domain_bridge.yaml`.

3. **Run the dashboard:**

   ```bash
   ros2 launch rmf_manager_dashboard dashboard.launch.py
   ```

   * Task Manager: [http://localhost:5005](http://localhost:5005)
   * Visualizer: [http://localhost:8080](http://localhost:8080)

> Tip: If you change configs, `--symlink-install` lets you rerun without rebuilding.

---

## Topics & Services

### Topics

* `/rmf/state` — `rmf_manager_msgs/msg/RobotState` (published by core)
* `/rmf/event` — `rmf_manager_msgs/msg/RobotEvent` (core emits task lifecycle events)
* `/rmf/command` — `rmf_manager_msgs/msg/RobotCommand` (UI or external apps can publish)

### Services (provided by core)

* `/schedule_patrol` — `rmf_manager_msgs/srv/SchedulePatrol`
* `/schedule_delivery` — `rmf_manager_msgs/srv/ScheduleDelivery`
* `/schedule_goto` — `rmf_manager_msgs/srv/ScheduleGoTo`
* `/cancel_task` — `rmf_manager_msgs/srv/CancelTask`
* `/cancel_all` — `rmf_manager_msgs/srv/CancelAll`
* `/list_tasks` — `rmf_manager_msgs/srv/ListTasks`
* `/set_resume_after_charge` — `rmf_manager_msgs/srv/SetResumeAfterCharge`
* `/trigger_delivery` — `rmf_manager_msgs/srv/TriggerDelivery`
* (Dashboard also uses `MoveTask` if present in `rmf_manager_msgs`)

### Example commands

```bash
# Go to a single waypoint
ros2 service call /schedule_goto rmf_manager_msgs/srv/ScheduleGoTo "{goal: a}"

# Patrol a set of waypoints for 2 loops
ros2 service call /schedule_patrol rmf_manager_msgs/srv/SchedulePatrol \
  "{waypoints: [a, b, station], loops: 2}"

# Cancel all tasks
ros2 service call /cancel_all rmf_manager_msgs/srv/CancelAll "{}"
```

---

## `rmf_manager_core` Parameters (high-value subset)

* Identification & frames

  * `robot_name` (string, default: node name)
  * `map_name` (string, default: `unknown_map`)
  * `floor_name` (string, default: `floor_0`)
  * `global_frame` (string, default: `map`)
  * `robot_base_frame` (string, default: `base_link`)
* Graph files

  * `param_yaml_path` (string, **required**): path to transform params (`floor_params.yaml`)
  * `building_yaml_path` (string, **required**): path to building/graph (`building.yaml`)
* Docking

  * `dock_action_name` (`/dock_robot`), `undock_action_name` (`/undock_robot`)
  * `home_dock_id` (`home_dock`), `home_vertex` (`station`)
* Navigation behavior

  * `handoff_distance` (0.8), `handoff_min_edge_ratio` (0.45), `handoff_min_interval` (0.25 s)
  * `goal_timeout_sec` (180.0), `arrival_tolerance` (0.18)
  * `viz_update_hz` (1.0) — publish rate for `/rmf/state`
* Collision / preskip

  * `local_costmap_topic` (`/local_costmap/costmap`)
  * `collision_detector_topic` (`/colision_detector`)
  * `collision_block_return` (true)
  * `collision_preskip_enable` (true)
  * `collision_preskip_check_dist` (1.5 m)
  * `collision_preskip_radius_m` (0.5 m)
  * `collision_preskip_occ_threshold` (50)
  * `collision_preskip_min_fraction` (0.05)
  * `collision_preskip_edge_ratio` (0.85)
  * `costmap_max_age_sec` (2.0)

These are set directly in `rmf_manager_core/launch/rmf_manager.launch.py` (no `DeclareLaunchArgument` indirection).

---

## Web Dashboard

* **Task Manager** (`http://<host>:5005/`)

  * Schedule Patrol/Delivery/Go-To
  * Reorder (move top/up/down/bottom, set index), Cancel/Cancel All
  * Trigger delivery stages when `wait_for_trigger` is set
  * “Open Map Visualizer” button links to the visualizer port

* **Visualizer** (`http://<host>:8080/`)

  * Background image from `building.yaml` `drawing.filename`
  * Edges/vertices overlay
  * Live robot pose (arrow)
  * Planned path polyline
  * Zoom/pan (mouse/keyboard), label sizing controls, refresh

**Config parameters for the dashboard node**

* `ui_host` (`0.0.0.0`), `ui_port` (`5005`)
* `ui_viz_host` (`0.0.0.0`), `ui_viz_port` (`8080`)
* `viz_update_hz` (1.0)
* `param_yaml_path`, `building_yaml_path` (for graph & image)

---

## Domain Bridge

A ready-to-use config is installed to:

```
<install>/share/rmf_manager_core/config/domain_bridge.yaml
```

Default bridges:

* Domains: `from_domain: 0` ↔ `to_domain: 1`
* Topics: `/rmf/state`, `/rmf/event`, `/rmf/command`
* Services: all task services listed above

**Change domains or add bridges** by editing that YAML.

> If you see `error parsing the file '--config': file does not exist`, make sure you launched **from the installed package** (after `colcon build`) and didn’t delete the installed `config/` directory.

---

## Troubleshooting

* **TF failed: “map … does not exist”**

  * Ensure your TF tree publishes `map -> base_link` (or set `global_frame`/`robot_base_frame` to match your system).
* **No vertices/edges in the UI**

  * Check `building.yaml` path and structure (`levels`, `vertices`, `lanes`) and the `floor_params.yaml` transform. Confirm the launch parameters point to the correct files.
* **Visualizer shows no background**

  * Verify `drawing.filename` is present and the image path resolves relative to `building.yaml`.
* **Dashboard template not found**

  * Run from `install` after a build: the node resolves templates under `share/rmf_manager_dashboard/web`.
* **Domain bridge crashes on startup**

  * Confirm the file exists under `share/rmf_manager_core/config/domain_bridge.yaml` in your **install** tree.
* **Clean Ctrl+C exit**

  * The dashboard node handles SIGINT and shuts down rclpy idempotently; if you see shutdown warnings, ensure you’re using the updated script.

---


## Using HTTP

* Read current mode:

```bash
curl http://localhost:8790/mode
```

```json
{ "mode_code": 3, "mode_name": "paused" }
```

* Set mode:

```
# Pause now
curl "http://localhost:8790/set_mode?code=3"

# Resume work (continue the last task)
curl "http://localhost:8790/set_mode?code=2"

# Go home immediately (preempts current task)
curl "http://localhost:8790/set_mode?code=6"

# Emergency stop: cancels all; blocks everything until idle
curl "http://localhost:8790/set_mode?code=5"

# Clear emergency & fully idle
curl "http://localhost:8790/set_mode?code=0"
```



* Read full state:

```bash
curl http://localhost:8790/state
```

```json
{
  "name": "orca_01",
  "fleet_name": "orca_01",
  "mode": 2,
  "battery_percent": 0.73,
  "location": {
    "map": "warehouse_A",
    "x": 12.345678,
    "y": -3.210000,
    "yaw": 1.570796,
    "vertexs": ["v3","v4","dock_entry","station"]
  },
  "location_wcs": {
    "lat": 42.697708,
    "lon": 23.321868,
    "alt": 555.100000,
    "orientation": { "system": "enu", "x": 0.01, "y": 0.02, "z": 0.70, "w": 0.71 } // ENU: x = East y = North z = Up and quaternions data
  }
}
```

### **Modes**

**0 – idle**
  No work is running. Setting this (`set_mode?code=0`) cancels everything and clears **paused**/**emergency** latches. Also shown briefly while *undocking*.

**1 – charging**
  Robot is docked (`docked_ == true`). You cannot set this manually (it’s derived from state).

**2 – moving**
  A task is active and the robot is free to execute. You can “resume” from pause with `set_mode?code=2`.

**3 – paused**
  Manual hold. When you set `code=3`, the node cancels the current **Nav2** goal **but keeps the current task** in place; when you later `code=2` it continues that same task/segment (it will resend the Nav2 goal).

**4 – waiting**
  There’s work **queued** but nothing active right now (e.g., between tasks). Not user-settable—derived from queue state.

**5 – emergency**
  Hard stop. `set_mode?code=5` cancels **everything** and blocks all activity. While emergency is active, **the only allowed change is back to `idle (0)`**. After you switch to idle, you can pick other modes again.

**6 – going_home**
  Return-to-dock sequence. `set_mode?code=6` immediately queues a “return” task **at the front** and preempts current motion. (Also shown whenever a “return” task is running.)

**7 – docking**
  In the docking action (set by the docking routine). Not user-settable.

**8 – request_error**
  Fallback if the mode computation hits an unexpected exception. Not user-settable.

### Manual blocks / rules


- You **cannot** set **1 (charging)**, **7 (docking)**, **8 (request_error)**.
- While **5 (emergency)** is active, the **only** accepted set is **0 (idle)**.
- **3 (paused)** cancels the current Nav2 goal but preserves the task; **2 (moving)** resumes it.
- **6 (going_home)** preempts and sends the robot home immediately.


### `/mode`

| Name      | Description                       | Data type |
| --------- | --------------------------------- | --------- |
| mode_code | Mode code (0–8)                   | int       |
| mode_name | Mode name (`idle`, `moving`, etc) | string    |

### `/state`

| Name                            | Description                                       | Data type     |
| ------------------------------- | ------------------------------------------------- | ------------- |
| name                            | Robot name                                        | string        |
| fleet_name                      | Fleet/group name                                  | string        |
| mode                            | Current mode code (0–8)                           | int           |
| battery_percent                 | Battery level (0.0–1.0)                           | double | null |
| location                        | Map-frame pose block                              | object        |
| location.map                    | Map name                                          | string        |
| location.x                      | X position relative to map origin (m)             | double | null |
| location.y                      | Y position relative to map origin (m)             | double | null |
| location.yaw                    | Yaw relative to map origin (rad)                  | double | null |
| location.vertexs                | Occupied/traversed graph vertices                 | array<string> |
| location_wcs                    | World-coordinates block (WGS84 + ENU orientation) | object        |
| location_wcs.lat                | Latitude                                          | double | null |
| location_wcs.lon                | Longitude                                         | double | null |
| location_wcs.alt                | Altitude (m)                                      | double | null |
| location_wcs.orientation        | Orientation quaternion (ENU)                      | object        |
| location_wcs.orientation.system | Reference frame — `enu`                           | string        |
| location_wcs.orientation.x      | Quaternion x                                      | double | null |
| location_wcs.orientation.y      | Quaternion y                                      | double | null |
| location_wcs.orientation.z      | Quaternion z                                      | double | null |
| location_wcs.orientation.w      | Quaternion w                                      | double | null |


---

## Development Notes

* `rmf_manager_core` is a single C++ node. It uses Nav2’s `NavigateToPose` and optional OpenNav Docking actions. Path planning over the graph uses Dijkstra and yields a vertex sequence; each vertex is sent to Nav2 with a computed heading. The “handoff radius” allows early transition to the next leg once within radius, improving overall motion.
* “Preskip” inflates the handoff radius if an obstacle fraction around the target is high in the local costmap. Tune `preskip_*` parameters per robot.




echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null
sudo apt update

Then either:

install the plugin with: sudo apt install zenoh-plugin-ros2dds.
install the standalone executable with: sudo apt install zenoh-bridge-ros2dds.

sudo apt install zenohd


sudo apt install -y ros-humble-cv-bridge python3-opencv




export INFLUX_TOKEN=replace_with_a_long_random_token
curl -X POST "http://127.0.0.1:8086/api/v2/delete?org=orca&bucket=rmf" \
  -H "Authorization: Token $INFLUX_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "start":"1970-01-01T00:00:00Z",
    "stop":"2100-01-01T00:00:00Z",
    "predicate":""
  }'
