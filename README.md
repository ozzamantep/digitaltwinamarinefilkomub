# 🌊 Digital Twin AUV - Amarine FILKOM Universitas Brawijaya
### Real-Time 6-DOF Autonomous Underwater Vehicle Digital Twin & Telemetry Software

Official repository for the Autonomous Underwater Vehicle (AUV) Digital Twin developed for **Amarine FILKOM UB (Universitas Brawijaya)**. Features hardware-in-the-loop (HIL) and software-in-the-loop (SITL) subsea simulation, closed-loop PID control, online system identification (ARMAX with Recursive Least Squares), and 3D subsea visualization.

![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-blue?logo=ros)
![Electron](https://img.shields.io/badge/Platform-Native%20Desktop%20%28Windows%29-47848F?logo=electron)
![ThreeJS](https://img.shields.io/badge/Render-Three.js%20%2F%20WebGL-black?logo=three.js)
![GPU](https://img.shields.io/badge/GPU-RTX%204050%20Hardware%20Accel-76B900?logo=nvidia)

---

## 🏗️ Architecture

```
┌──────────────────────────────────────────────┐                ┌────────────────────────────────────────┐
│         SUBSEA ROBOT / SIMULATOR             │                │       DIGITAL TWIN DESKTOP SOFTWARE    │
│       (Jetson Nano / WSL2 Gazebo)            │                │         (Electron + React Three.js)    │
├──────────────────────────────────────────────┤  WebSocket     ├────────────────────────────────────────┤
│ • ROS 2 Humble / Jazzy Nodes                 │ ─────────────> │ • 3D CAD AUV Viewport & Thruster Renders│
│ • Sensor IMU, Depth (MS5837), DVL, Battery   │  (Port 9090)   │ • ARMAX / RLS Online Adaptive SysID    │
│ • Subsea Camera Feed (/camera/image_raw)     │                │ • Closed-Loop PID Flight Tuning        │
│ • 6-Thruster Allocation Matrix (TAM)         │ <───────────── │ • YOLO Real-Time Frustum Target Bounding│
└──────────────────────────────────────────────┘   /cmd_vel     └────────────────────────────────────────┘
```

---

## 🛥️ Custom 6-Thruster AUV Configuration

- **4x Horizontal Vectored Thrusters**: 4 corners mounted at 45° angles for full surge, sway, and yaw vectoring.
- **2x Vertical Ducted Thrusters**: Mounted vertically inside aerodynamic yellow nose and tail cowlings for heave (dive/surface) and pitch trim.
- **Dual Stacked Carbon Fiber Battery Tubes**: 4 cylindrical pressure hulls (2 stacked on port, 2 on starboard).
- **Clear Acrylic Electronics Enclosure**: Houses Jetson Nano, flight controller, status telemetry LEDs, and power buses.

---

## 🚀 Quick Start

### 1. Run Native Desktop Software (Windows)
Double-click `Launch_Desktop_App.bat` or run:
```bash
npm install
npm run desktop
```

### 2. Run Gazebo 3D Simulation in WSL2
Double-click `Launch_Gazebo_WSL.bat` or inside WSL Ubuntu 22.04:
```bash
cd simulator
./setup_wsl_gazebo.sh
```

---

## 📡 ROS 2 Subsea Topics

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/odom` | `nav_msgs/msg/Odometry` | Subsea 3D position, orientation quaternion, linear/angular velocity |
| `/imu/data` | `sensor_msgs/msg/Imu` | Roll, pitch, yaw orientation & 3-axis linear acceleration |
| `/depth` | `sensor_msgs/msg/FluidPressure` | Water pressure & depth from MS5837 barometer sensor |
| `/dvl/range` | `sensor_msgs/msg/Range` | Acoustic DVL distance to pool bottom |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | 4S LiPo voltage (16V), current draw, remaining capacity |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 4-DOF velocity commands (surge, sway, heave, yaw) |
| `/thruster_commands` | `std_msgs/msg/Float64MultiArray` | Direct 6-channel thruster PWM/Effort outputs |
| `/thruster_outputs` | `std_msgs/msg/Float64MultiArray` | Real-time thruster RPM feedback from hardware |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Front subsea camera video stream |

---

## 🛠️ Tech Stack

- **Core & Desktop:** Electron 44, React 19, Vite, Vanilla CSS Design System
- **3D Graphics & Physics:** Three.js, React Three Fiber, Drei, 6-DOF Hydrodynamic Controller
- **System Identification:** ARMAX Discrete Polynomials, Recursive Least Squares (RLS, $\lambda=0.985$)
- **Robotics Middleware:** ROS 2 Humble / Jazzy, roslibjs, Gazebo 11 Classic (WSLg)

---

Developed with ❤️ for **Amarine FILKOM Universitas Brawijaya**.
