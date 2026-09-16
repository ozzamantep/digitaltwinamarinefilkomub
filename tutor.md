# Connecting the Digital Twin to the Physical AUV

This guide connects the desktop Digital Twin to the Jetson onboard the AUV using ROS 2 and rosbridge WebSocket.

## 1. Safety Before Connecting

1. Keep the AUV disarmed before connecting the application.
2. Keep a physical emergency stop and a battery disconnect accessible.
3. Test commands with propellers removed or with the vehicle restrained before an in-water test.
4. Use `/cmd_vel` through the flight-controller safety node. Do not connect the dashboard directly to ESC outputs for the first test.

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
cd ~/digitaltwin
bash jetson/setup_rosbridge.sh
```

The script installs `rosbridge_server` and compressed image transport.

## 4. Start the ROS 2 Bridge

Each time the AUV system is used, start ROS 2 and the bridge:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 launch digitaltwin digitaltwin.launch.py
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
ros2 topic echo /imu/data --once
ros2 topic echo /depth --once
ros2 topic echo /battery_state --once
```

The application expects these ROS 2 topics:

| Topic | Type | Purpose |
|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | Position, orientation, and velocity |
| `/imu/data` | `sensor_msgs/msg/Imu` | Attitude and acceleration |
| `/depth` | `sensor_msgs/msg/FluidPressure` | Pressure-derived depth |
| `/dvl/range` | `sensor_msgs/msg/Range` | Altitude above the pool floor |
| `/sonar/front/range` | `sensor_msgs/msg/Range` | Forward collision clearance |
| `/sonar/rear/range` | `sensor_msgs/msg/Range` | Rear collision clearance |
| `/sonar/left/range` | `sensor_msgs/msg/Range` | Port-side collision clearance |
| `/sonar/right/range` | `sensor_msgs/msg/Range` | Starboard-side collision clearance |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | Battery health |
| `/thruster_outputs` | `std_msgs/msg/Float64MultiArray` | Signed normalized thruster feedback |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Forward camera stream |

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

On Jetson, monitor the velocity command topic:

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

Possible commands are `OPEN`, `CLOSE`, `GRASP`, and `RELEASE`.

## 8. Synchronizing a Different Start Position

The physical AUV can start anywhere in the pool. The Digital Twin follows the newest `/odom` pose, depth, and IMU telemetry.

Use one consistent pool coordinate frame:

1. Define a pool origin before the run, such as one start-zone corner.
2. Publish `/odom` relative to that origin.
3. Keep the same axis convention for localization, mission waypoints, and the Digital Twin.
4. Do not reset the Digital Twin to a fixed simulator location while in live mode.

If DVL bottom lock is unavailable, use IMU dead reckoning only temporarily; position drift grows quickly underwater. Restore DVL, vision, or external localization before an autonomous run.

## 9. Pre-Water Checklist

- `ros2 topic echo /odom --once` returns a valid pose.
- `/imu/data`, `/depth`, and `/battery_state` update continuously.
- All sonar and DVL ranges are finite and match measured distances.
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

### Telemetry is not updating

Check each ROS 2 topic locally on Jetson. rosbridge can be connected even when upstream sensor nodes are not publishing.

### The Digital Twin pose is wrong

Check the `/odom` coordinate frame and axis conversion. The twin expects pool position and depth in a consistent NED-style convention.

### Commands arrive but the AUV does not move

Keep the vehicle disarmed until topic direction is verified, then check the flight-controller node, arming state, kill switch, ESC power, and `/cmd_vel` subscriber.