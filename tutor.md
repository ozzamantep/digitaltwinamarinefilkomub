# Connecting the Digital Twin to the Physical AUV

This guide connects the desktop Digital Twin to the Jetson onboard the AUV using ROS 2 and rosbridge WebSocket.

## 1. Safety Before Connecting

1. Keep the AUV disarmed before connecting the application.
2. Keep a physical emergency stop and a battery disconnect accessible.
3. Test commands with propellers removed or with the vehicle restrained before an in-water test.
4. Run `backend/sauvc26_code/manual_bridge.py` on the Jetson for manual piloting - it is the node that subscribes `/cmd_vel` and `/gripper/command` and forwards them to MAVROS. It cannot run at the same time as `final.py`/`qualification.py` - `control_lock.py` enforces this automatically (a PID lock file) and the second node will fail to start with a clear error instead of silently fighting for control. Do not connect the dashboard directly to ESC outputs for the first test.

## 2. Network Setup

Connect the laptop and Jetson to the same Wi-Fi router or Ethernet network.

On the Jetson, find its IP address:

```bash
hostname -I
```

Example output:

```text
192.168.1.100
```

Use this address in the Digital Twin connection field.

## 3. Install rosbridge on Jetson

Run this once on the Jetson:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/digitaltwinamarinefilkomub   # the folder where this repo was cloned - check with `pwd`/`ls ~` if unsure
bash backend/setup_rosbridge.sh
```

The script installs `rosbridge_server` and compressed image transport.

**Note:** this repo is a plain set of scripts/launch files, NOT an installed ROS 2 package - there is no `colcon build` step and no package named `digitaltwin` registered with `ros2`. Every command below is run either as a direct Python script (`python3 ...`) or a path-based `ros2 launch <file>.launch.py` (both work without building/sourcing an install workspace). Do NOT use `ros2 run digitaltwin ...` - it will fail with `Package 'digitaltwin' not found`.

## 3b. Start the MAVROS Telemetry Bridge (REQUIRED for position, battery, depth & front sonar sync)

`/odom`, `/battery_state`, `/depth`, and `/sonar/front/range` have no publisher on the real vehicle by default - MAVROS only exposes `/mavros/local_position/pose`, `/velocity_local`, `/mavros/battery`, `/mavros/imu/static_pressure`, and the distance-sensor topic. Without this node running, the Digital Twin's 3D position never moves, the real low-battery emergency-surface safety check never sees real voltage, and the real depth/front-sonar safety checks never see real sensor data:

```bash
cd ~/digitaltwinamarinefilkomub
source /opt/ros/$ROS_DISTRO/setup.bash
python3 backend/sauvc26_code/odom_bridge.py
```

This is a read-only telemetry bridge (no actuation) - safe to run alongside any other node, always.

## 3c. Start the Camera Driver (REQUIRED for the live video feed)

`/camera/image_raw/compressed` has no publisher yet on the real vehicle. Use the standard ROS2 `v4l2_camera` driver + `image_transport` republish (no custom code needed):

```bash
cd ~/digitaltwinamarinefilkomub
bash backend/start_camera.sh /dev/video0
```

Pass a different device path as the first argument if the camera isn't `/dev/video0`.

## 4. Start the ROS 2 Bridge

Each time the AUV system is used, start ROS 2 and the bridge:

```bash
cd ~/digitaltwinamarinefilkomub
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 launch backend/launch_digitaltwin.launch.py
```

The default endpoint is:

```text
ws://JETSON_IP:9090
```

For example:

```text
ws://192.168.1.100:9090
```

Verify that the port is listening:

```bash
ss -lnt | grep 9090
```

## 5. Confirm Telemetry Topics

Before opening the desktop application, confirm that the physical AUV publishes the required data:

```bash
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /mavros/imu/data --once
ros2 topic echo /depth --once
ros2 topic echo /battery_state --once
```

The application expects these ROS 2 topics:

| Topic | Type | Purpose |
|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | Position, orientation, and velocity |
| `/mavros/imu/data` | `sensor_msgs/msg/Imu` | Attitude and acceleration (direct from Pixhawk via MAVROS) |
| `/depth` | `sensor_msgs/msg/FluidPressure` | Pressure-derived depth - MS5837/Bar30 wired directly to Pixhawk over I2C (per wiring schematic), bridged from `/mavros/imu/static_pressure` by `odom_bridge.py` |
| `/dvl/range` | `sensor_msgs/msg/Range` | Altitude above the pool floor - **no DVL on the real hull**, floor-lock safety has no real data source |
| `/sonar/front/range` | `sensor_msgs/msg/Range` | Forward collision clearance - **the only sonar the real hull actually has**, wired into the Pixhawk rangefinder input and bridged by `odom_bridge.py` (verify the exact MAVROS distance-sensor topic name matches `MAVROS_DISTANCE_SENSOR_TOPIC` in that file) |
| `/sonar/rear/range` | `sensor_msgs/msg/Range` | Rear collision clearance - **no physical sensor**, rear-wall safety has no real data source |
| `/sonar/left/range` | `sensor_msgs/msg/Range` | Port-side collision clearance - **no physical sensor** |
| `/sonar/right/range` | `sensor_msgs/msg/Range` | Starboard-side collision clearance - **no physical sensor** |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | Battery health |
| `/thruster_outputs` | `std_msgs/msg/Float64MultiArray` | Signed normalized thruster feedback |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Forward camera stream |
| `/mission_state` | `std_msgs/msg/String` | Real mission events (flare hit, payload dropped) reported back from the vehicle |
| `/mavros/state` | `mavros_msgs/msg/State` | Real armed/flight-mode feedback from the Pixhawk |

The dashboard's arm/disarm and flight-mode buttons also call the real `/mavros/cmd/arming` and `/mavros/set_mode` MAVROS services directly whenever connected live - they are not just local UI state.

## 6. Start the Digital Twin

On the Windows laptop:

```powershell
cd C:\Users\ACER\Downloads\digitaltwin
npm run dev
```

Or run the desktop application:

```powershell
Launch_Desktop_App.bat
```

In the Digital Twin header:

1. Enter the Jetson IP address.
2. Click `Connect`.
3. Confirm the status changes to `JETSON ORIN LINKED`.
4. Confirm live depth, heading, battery, and position update before arming.

The application automatically leaves demo mode, subscribes to the telemetry topics, and publishes control commands after connecting.

## 7. Verify Commands Safely

On Jetson, make sure `manual_bridge.py` is running (not the autonomous mission nodes), then monitor the velocity command topic:

```bash
ros2 topic echo /cmd_vel
```

Operate the dashboard with the AUV disarmed. Confirm these mappings:

| Command | ROS field |
|---|---|
| Surge forward/backward | `linear.x` |
| Sway right/left | `linear.y` |
| Heave down/up | `linear.z` |
| Yaw right/left | `angular.z` |

The gripper uses:

```bash
ros2 topic echo /gripper/command
```

Possible commands are `OPEN`, `CLOSE`, `GRASP`, and `RELEASE`. On real hardware only `OPEN`/`RELEASE` actuate the servo (single-channel, release-only) - `manual_bridge.py` logs a warning and ignores `CLOSE`/`GRASP` since no hold position is calibrated.

## 8. Synchronizing a Different Start Position

The physical AUV can start anywhere in the pool. The Digital Twin follows the newest `/odom` pose, depth, and IMU telemetry.

Use one consistent pool coordinate frame:

1. Define a pool origin before the run, such as one start-zone corner.
2. Publish `/odom` relative to that origin.
3. Keep the same axis convention for localization, mission waypoints, and the Digital Twin.
4. Do not reset the Digital Twin to a fixed simulator location while in live mode.

If DVL bottom lock is unavailable, use IMU dead reckoning only temporarily; position drift grows quickly underwater. Restore DVL, vision, or external localization before an autonomous run.

## 9. Pre-Water Checklist

- `ros2 topic echo /odom --once` returns a valid pose (requires `odom_bridge.py` running).
- `/mavros/imu/data`, `/battery_state`, and `/depth` update continuously (`/depth` requires `odom_bridge.py` running and depends on `/mavros/imu/static_pressure` reporting the water pressure sensor - verify readings rise as the vehicle is submerged, not just static air pressure).
- `/sonar/front/range` is finite and matches measured distance (the only sonar the real hull has - rear/left/right/DVL have no physical sensor yet, so collision safety only actively protects the front). Verify the MAVROS distance-sensor topic name matches `MAVROS_DISTANCE_SENSOR_TOPIC` in `odom_bridge.py` before trusting this reading.
- Dashboard status is `JETSON ORIN LINKED`.
- `/cmd_vel` direction is verified while disarmed.
- Emergency stop sends zero velocity and zero thruster command.
- First in-water run uses low command limits and an operator ready to disarm.

## Troubleshooting

### The dashboard stays disconnected

Check the Jetson IP, network reachability, and rosbridge port:

```bash
ping JETSON_IP
ss -lnt | grep 9090
```

### `Package 'digitaltwin' not found`

This repo has no installed ROS 2 package named `digitaltwin` - `ros2 run digitaltwin ...` will always fail with this error. Use the path-based/direct-script forms shown in this guide instead: `ros2 launch backend/launch_digitaltwin.launch.py` (not `ros2 launch digitaltwin digitaltwin.launch.py`) and `python3 backend/sauvc26_code/odom_bridge.py` (not `ros2 run digitaltwin odom_bridge`). Both must be run from the repo's root folder (`cd ~/digitaltwinamarinefilkomub` or wherever it was cloned).

### Telemetry is not updating

Check each ROS 2 topic locally on Jetson. rosbridge can be connected even when upstream sensor nodes are not publishing.

### The Digital Twin pose is wrong

Check the `/odom` coordinate frame and axis conversion. The twin expects pool position and depth in a consistent NED-style convention.

### Commands arrive but the AUV does not move

Keep the vehicle disarmed until topic direction is verified, then check: is `manual_bridge.py` actually running on the Jetson (it is the only node that subscribes `/cmd_vel` for the real vehicle - the autonomous mission nodes ignore it), the arming state, kill switch, ESC power, and GUIDED mode.