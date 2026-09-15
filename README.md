# 🌊 Digital Twin AUV — Amarine FILKOM Universitas Brawijaya

### Real-Time 6-DOF Autonomous Underwater Vehicle Digital Twin & Telemetry Software

[![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-blue?logo=ros)](https://docs.ros.org/)
[![Electron](https://img.shields.io/badge/Platform-Native%20Desktop%20(Windows)-47848F?logo=electron)](https://www.electronjs.org/)
[![React](https://img.shields.io/badge/React-19-61DAFB?logo=react)](https://react.dev/)
[![Three.js](https://img.shields.io/badge/Render-Three.js%20%2F%20WebGL-black?logo=three.js)](https://threejs.org/)
[![Vite](https://img.shields.io/badge/Build-Vite-646CFF?logo=vite)](https://vitejs.dev/)
[![GPU](https://img.shields.io/badge/GPU-RTX%204050%20Accelerated-76B900?logo=nvidia)](https://www.nvidia.com/)
[![License](https://img.shields.io/badge/License-Academic%20Research-green)]()

> **Official repository** for the 6-DOF AUV Digital Twin developed for **Tim Amarine FILKOM UB (Universitas Brawijaya)**. Features full nonlinear Fossen hydrodynamics, hardware-in-the-loop (HIL) and software-in-the-loop (SITL) subsea simulation, closed-loop PID control, online system identification (Box-Jenkins with pseudo-linear Recursive Least Squares), and real-time 3D subsea visualization.

---

## 🏗️ System Architecture

```
┌──────────────────────────────────────────────┐                ┌────────────────────────────────────────┐
│         SUBSEA ROBOT / SIMULATOR             │                │       DIGITAL TWIN DESKTOP SOFTWARE    │
│       (Jetson Nano / WSL2 Gazebo)            │                │         (Electron + React Three.js)    │
├──────────────────────────────────────────────┤  WebSocket     ├────────────────────────────────────────┤
│ • ROS 2 Humble / Jazzy Nodes                 │ ────────────►  │ • 3D CAD AUV Viewport & Thruster Viz  │
│ • Sensor IMU, Depth (MS5837), DVL, Battery   │  (Port 9090)   │ • BJ / RLS Online Adaptive SysID      │
│ • Subsea Camera Feed (/camera/image_raw)     │                │ • Closed-Loop PID Flight Tuning       │
│ • 6-Thruster Allocation Matrix (TAM)         │ ◄────────────  │ • 15-State EKF Sensor Fusion          │
└──────────────────────────────────────────────┘   /cmd_vel     └────────────────────────────────────────┘
```

### Data Flow Pipeline (50 Hz Loop)

```
Sensor Data ──► EKF Predict ──► EKF Update ──► SysID (RLS) ──► PID Controller
                    │               │               │                │
               IMU 100Hz      Depth 20Hz     Parameter θ         cmd_vel
                                DVL 10Hz                             │
                                                                     ▼
                                                            TAM Thruster Allocation
                                                                     │
                                                                     ▼
                                                          Fossen 6-DOF Dynamics
                                                          (RK4 Integration @ 50Hz)
                                                                     │
                                                                     ▼
                                                          3D WebGL Viewport Render
```

---

## 🛥️ Custom 6-Thruster AUV Configuration

| Component | Specification |
|:----------|:-------------|
| **4× Horizontal Vectored Thrusters** | T200 at 45° corners for full surge, sway, and yaw vectoring |
| **2× Vertical Ducted Thrusters** | Inside aerodynamic nose and tail cowlings for heave & pitch |
| **Battery System** | 4× cylindrical pressure hulls, dual stacked (4S LiPo 16V) |
| **Electronics** | Clear acrylic enclosure: Jetson Nano, flight controller, status LEDs |
| **Dry Mass** | 11.5 kg |
| **Displaced Volume** | 11.51 L (0.01151 m³, near-neutral buoyancy) |
| **Dimensions** | 540mm × 280mm × 240mm |

---

## 🧮 Key Mathematical Models

This Digital Twin implements the following physics models from Fossen (2021):

### 6-DOF Fossen Equation of Motion
```
M ν̇ + C(ν)ν + D(ν_r)ν_r + g(η) = τ_thruster + τ_env + τ_pinn
```

### Mass Matrix (M = M_RB + M_A)

| DOF | M_RB | M_A (Added Mass) | **M_total** | **M⁻¹** |
|:----|:-----|:-----------------|:------------|:---------|
| Surge | 11.5 kg | 5.5 kg | **17.0 kg** | 0.05882 |
| Sway | 11.5 kg | 12.7 kg | **24.2 kg** | 0.04132 |
| Heave | 11.5 kg | 14.6 kg | **26.1 kg** | 0.03831 |
| Roll | 0.12 kg·m² | 0.12 kg·m² | **0.24 kg·m²** | 4.16667 |
| Pitch | 0.22 kg·m² | 0.12 kg·m² | **0.34 kg·m²** | 2.94118 |
| Yaw | 0.24 kg·m² | 0.12 kg·m² | **0.36 kg·m²** | 2.77778 |

### Hydrodynamic Damping Coefficients

| DOF | Linear (D_L) | Quadratic (D_Q) |
|:----|:-------------|:----------------|
| Surge | Xu = 4.03 N·s/m | Xuu = 18.18 N·s²/m² |
| Sway | Yv = 6.22 N·s/m | Yvv = 21.66 N·s²/m² |
| Heave | Zw = 5.18 N·s/m | Zww = 36.99 N·s²/m² |
| Roll/Pitch/Yaw | 0.07 N·m·s/rad | 1.55 N·m·s²/rad² |

### Hydrostatic Restoring Forces
```
Weight:    W = 11.5 × 9.80665 = 112.78 N
Buoyancy:  B = 996.78 × 9.80665 × 0.01151 = 112.49 N   (at 26°C pool)
Net Force: ΔF = -0.29 N  (near-neutral buoyancy)

Righting Moment (CB 25mm above CG):
  Roll:  K_g = +2.81 × sin(φ) N·m
  Pitch: M_g = +2.81 × sin(θ) N·m
```

### Thruster Allocation Matrix (6×6)
```
           T1       T2       T3       T4       T5       T6
TAM = [  +0.707,  +0.707,  +0.707,  +0.707,   0.000,   0.000 ]  ← Surge (X)
      [  -0.707,  +0.707,  +0.707,  -0.707,   0.000,   0.000 ]  ← Sway  (Y)
      [   0.000,   0.000,   0.000,   0.000,  -1.000,  -1.000 ]  ← Heave (Z)
      [   0.000,   0.000,   0.000,   0.000,   0.000,   0.000 ]  ← Roll  (K)
      [   0.000,   0.000,   0.000,   0.000,  +0.180,  -0.180 ]  ← Pitch (M)
      [  -0.177,  +0.177,  -0.177,  +0.177,   0.000,   0.000 ]  ← Yaw   (N)
```

> 📘 **Full mathematical derivations with step-by-step numerical calculations available in [`THESIS_AND_TECHNICAL_ARCHITECTURE_GUIDE.md`](THESIS_AND_TECHNICAL_ARCHITECTURE_GUIDE.md)**

---

## 🧠 Digital Twin Core Engines

| Engine | Description | File |
|:-------|:-----------|:-----|
| **6-DOF Fossen Hydrodynamics** | Full nonlinear M, C, D, g matrices with RK4 integration | `src/dt-core/HydrodynamicsEngine.js` |
| **Vehicle Configuration** | Single source of truth for all 50+ physical parameters | `src/dt-core/VehicleConfig.js` |
| **15-State EKF** | Multi-rate sensor fusion (IMU 100Hz, Depth 20Hz, DVL 10Hz) with Mahalanobis outlier gating | `src/dt-core/StateEstimator.js` |
| **Box-Jenkins System Identification** | Online pseudo-linear RLS (λ=0.985) adapting 6 plant/noise parameters | `src/services/SystemIdentificationEngine.js` |
| **Competition Safety Supervisor** | Directional latches, active wall/floor repulsion, emergency surface, sensor fail-safe | `src/services/SafetySupervisor.js` |
| **PID Flight Controller** | Depth hold, heading lock, attitude stabilization with anti-windup | `src/services/AUVMotionController.js` |
| **Thruster Dynamics** | T200 lookup table, motor lag (τ=0.35s), voltage sag, PWM deadband | `src/services/ThrusterDynamicsModel.js` |
| **PINN Residual** | Physics-Informed Neural Network for unmodeled dynamics compensation | `src/dt-core/PINNResidual.js` |
| **Environment Model** | UNESCO water density, Gauss-Markov current turbulence, depth pressure | `src/dt-core/EnvironmentModel.js` |
| **Kinematics** | Quaternion integration, Euler↔quaternion, body↔world transforms | `src/dt-core/Kinematics.js` |

---

## 📡 ROS 2 Topics

| Topic | Type | Frequency | Description |
| :--- | :--- | :--- | :--- |
| `/odom` | `nav_msgs/Odometry` | 50 Hz | 3D position, orientation quaternion, velocities |
| `/imu/data` | `sensor_msgs/Imu` | 100 Hz | Roll, pitch, yaw & 3-axis accelerations |
| `/depth` | `sensor_msgs/FluidPressure` | 20 Hz | MS5837 barometer depth measurement |
| `/dvl/range` | `sensor_msgs/Range` | 10 Hz | Acoustic DVL altitude (distance to bottom) |
| `/sonar/front/range` | `sensor_msgs/Range` | ≥20 Hz | Forward collision distance |
| `/sonar/rear/range` | `sensor_msgs/Range` | ≥20 Hz | Rear collision distance |
| `/sonar/left/range` | `sensor_msgs/Range` | ≥20 Hz | Port-side collision distance |
| `/sonar/right/range` | `sensor_msgs/Range` | ≥20 Hz | Starboard-side collision distance |
| `/gripper/command` | `std_msgs/String` | Event | Gripper command: `OPEN`, `CLOSE`, `GRASP`, or `RELEASE` |
| `/battery_state` | `sensor_msgs/BatteryState` | 1 Hz | 4S LiPo voltage, current, remaining capacity |
| `/cmd_vel` | `geometry_msgs/Twist` | 50 Hz | 4-DOF velocity commands (surge, sway, heave, yaw) |
| `/thruster_commands` | `std_msgs/Float64MultiArray` | 50 Hz | 6-channel thruster PWM/effort outputs |
| `/thruster_outputs` | `std_msgs/Float64MultiArray` | 50 Hz | Real-time thruster RPM feedback |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | 30 Hz | Front subsea camera video stream |

### Physical Collision Safety

The safety state machine is `NORMAL -> CAUTION -> LOCKED -> EMERGENCY_SURFACE`. Four directional sonar streams use hysteresis: a wall lock engages at `0.55 m` and releases only above `1.20 m`. A locked direction ignores continued pilot throttle and commands active escape thrust away from the wall. Stale data older than `0.50 s` fails safe.

The downward DVL engages the floor lock at `0.37 m` and releases above `0.70 m`. It clears motor lag/RPM on T5-T6 and commands upward escape thrust, so holding Dive cannot keep loading the pool floor. Hull leak or critical battery (`<= 8%` or `<= 13.0 V`) latches emergency surface. IMU acceleration above `18 m/s²` or angular rate above `2.5 rad/s` triggers an all-axis emergency stop.

An IMU cannot measure distance to a wall or floor. It is used for attitude, motion detection, and impact fallback; preventive collision avoidance depends on correctly mounted and calibrated waterproof sonar/DVL sensors. Bench-test every topic direction and stopping distance at low thrust before operating near a pool wall or floor.

### Competition Perception & Interaction

- The sonar radar uses actual wall and arena-object range/bearing data. Contacts flash only when the rotating sweep crosses their bearing.
- Signal-only red buckets are excluded from acoustic sonar and labeled `CV ONLY`; the onboard camera/YOLO pipeline detects them because this vehicle has no signal receiver.
- The collision model uses a heading-aware `0.58 m x 0.32 m` oriented hull footprint plus vertical overlap. A flare falls only after real geometric contact, never from a generous waypoint radius.
- IMU, lowered forward camera, sonar housing, and vertical gripper are mounted on the lower centerline. Bubble jets show each T200 exhaust direction, including reverse and vertical thrust.
- Controller mappings: `Circle/B` opens or closes the gripper; `Cross/A` releases or grasps the ball. Releases enter the target drum only within `0.55 m`; otherwise the ball drops from the current gripper position.

### Real-Time Rendering Profile

The control, collision, IMU, and EKF loops remain at `20 Hz`. Sonar visualization runs at `10 Hz`, derived validation/OOD/health telemetry at `5 Hz`, and the secondary computer-vision viewport at `15 FPS`. Three.js components read live telemetry inside `useFrame` to avoid rebuilding the scene graph on every ROS packet.

---

## 🚀 Quick Start

### Prerequisites
- **Node.js** 18+ and **npm**
- **Windows 10/11** (for native desktop app via Electron)
- **WSL2 + Ubuntu 22.04** (optional, for Gazebo simulation)

### 1. Install Dependencies
```bash
npm install
```

### 2. Run Native Desktop App (Recommended)
Double-click `Launch_Desktop_App.bat` or:
```bash
npm run desktop
```

### 3. Run in Browser (Development Mode)
```bash
npm run dev
```
Open `http://localhost:5173` in browser.

### 4. Run Gazebo 3D Simulation in WSL2
Double-click `Launch_Gazebo_WSL.bat` or inside WSL:
```bash
cd simulator
./setup_wsl_gazebo.sh
```

---

## 📁 Project Structure

```
digitaltwin/
├── src/
│   ├── dt-core/                    ← Physics engine core (18 modules)
│   │   ├── VehicleConfig.js        ← All 50+ vehicle parameters
│   │   ├── HydrodynamicsEngine.js  ← 6-DOF Fossen dynamics + RK4
│   │   ├── Kinematics.js           ← Coordinate transforms
│   │   ├── EnvironmentModel.js     ← Water density, currents
│   │   ├── StateEstimator.js       ← 15-state EKF
│   │   └── ...
│   ├── services/                   ← Controllers & bridges (12 modules)
│   │   ├── AUVMotionController.js  ← PID flight controller
│   │   ├── SystemIdentificationEngine.js ← Box-Jenkins/RLS
│   │   ├── ThrusterDynamicsModel.js ← T200 motor model
│   │   ├── MockRosConnection.js    ← SITL ROS2 simulator
│   │   └── ...
│   └── components/                 ← React UI components
│       ├── 3d/                     ← Three.js 3D viewport
│       ├── dashboard/              ← Telemetry panels
│       └── thruster-test/          ← HIL T200 test page
├── electron/                       ← Electron desktop wrapper
├── jetson/                         ← Jetson Nano ROS2 configs
├── simulator/                      ← Gazebo simulation
├── tests/                          ← Unit & integration tests
└── public/                         ← Static assets & 3D models
```

---

## 🛠️ Tech Stack

| Layer | Technologies |
|:------|:------------|
| **Desktop Runtime** | Electron 44, Node.js |
| **Frontend** | React 19, Vite, Vanilla CSS Design System |
| **3D Graphics** | Three.js, React Three Fiber, Drei, WebGL |
| **Physics Engine** | Custom 6-DOF Fossen Hydrodynamics, RK4 Integrator |
| **System ID** | ARMAX / ARX / OE / Box-Jenkins, Recursive Least Squares (λ=0.985) |
| **State Estimation** | 15-State Extended Kalman Filter, Mahalanobis Gating |
| **Robotics** | ROS 2 Humble / Jazzy, roslibjs, rosbridge_server |
| **Simulation** | Gazebo Classic (WSLg), UUV Simulator Plugins |
| **Hardware** | NVIDIA Jetson Nano, BlueRobotics T200, MS5837, DVL |

---

## 📚 References

1. T. I. Fossen, *"Handbook of Marine Craft Hydrodynamics and Motion Control"*, 2nd ed., Wiley, 2021.
2. Berg, V., *"Development and Commissioning of a DP System for ROV SF 30k"*, NTNU, 2012.
3. Wu, G., *"Identification of Hydrodynamic Coefficients for an ROV"*, J. Ocean Engineering, 2018.
4. Blue Robotics, *"T200 Thruster Performance Data"*, bluerobotics.com, 2019.
5. UNESCO, *"International Equation of State of Seawater"*, Tech. Papers No. 36, 1980.

---

Developed with ❤️ for **Tim Amarine FILKOM Universitas Brawijaya**.
