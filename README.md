# 🌊 Digital Twin AUV — Amarine FILKOM Universitas Brawijaya

### Perangkat Lunak Digital Twin & Telemetri AUV (Autonomous Underwater Vehicle) 6-DOF Real-Time

[![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-blue?logo=ros)](https://docs.ros.org/)
[![Electron](https://img.shields.io/badge/Platform-Native%20Desktop%20(Windows)-47848F?logo=electron)](https://www.electronjs.org/)
[![React](https://img.shields.io/badge/React-19-61DAFB?logo=react)](https://react.dev/)
[![Three.js](https://img.shields.io/badge/Render-Three.js%20%2F%20WebGL-black?logo=three.js)](https://threejs.org/)
[![Vite](https://img.shields.io/badge/Build-Vite-646CFF?logo=vite)](https://vitejs.dev/)
[![GPU](https://img.shields.io/badge/GPU-RTX%204050%20Accelerated-76B900?logo=nvidia)](https://www.nvidia.com/)
[![License](https://img.shields.io/badge/License-Academic%20Research-green)]()

> **Repositori resmi** Digital Twin AUV 6-DOF yang dikembangkan untuk **Tim Amarine FILKOM UB (Universitas Brawijaya)**. Menghadirkan hidrodinamika nonlinear Fossen secara penuh, simulasi subsea hardware-in-the-loop (HIL) dan software-in-the-loop (SITL), kontrol PID closed-loop, identifikasi sistem online (Box-Jenkins dengan pseudo-linear Recursive Least Squares), dan visualisasi 3D subsea real-time.

---

## 🏗️ Arsitektur Sistem

```
┌──────────────────────────────────────────────┐                ┌────────────────────────────────────────┐
│         SUBSEA ROBOT / SIMULATOR             │                │       DIGITAL TWIN DESKTOP SOFTWARE    │
│       (Jetson Orin Nano / WSL2 Gazebo)       │                │         (Electron + React Three.js)    │
├──────────────────────────────────────────────┤  WebSocket     ├────────────────────────────────────────┤
│ • ROS 2 Humble / Jazzy Nodes                 │ ────────────►  │ • 3D CAD AUV Viewport & Thruster Viz  │
│ • Sensor IMU, Depth (MS5837), Sonar Depan, Battery │  (Port 9090)   │ • BJ / RLS Online Adaptive SysID      │
│ • Subsea Camera Feed (/camera/image_raw)     │                │ • Closed-Loop PID Flight Tuning       │
│ • 6-Thruster Allocation Matrix (TAM)         │ ◄────────────  │ • 15-State EKF Sensor Fusion          │
└──────────────────────────────────────────────┘   /cmd_vel     └────────────────────────────────────────┘
```

### Alur Data (Loop 50 Hz)

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

## 🛥️ Konfigurasi Kustom AUV 6-Thruster

| Komponen | Spesifikasi |
|:----------|:-------------|
| **4× Thruster Horizontal Vektor** | T200 di sudut 45° untuk vektor surge, sway, dan yaw penuh |
| **2× Thruster Vertikal Ducted** | Di dalam cowling hidung dan ekor aerodinamis untuk heave & pitch |
| **Sistem Baterai** | 4× tabung tekanan silinder, disusun dua tingkat (4S LiPo 16V) |
| **Elektronik** | Enclosure akrilik bening: Jetson Orin Nano, flight controller, LED status |
| **Massa Kering** | 22.5 kg (nominal; dapat dikonfigurasi 20-25 kg) |
| **Volume Displaced** | 22.54 L (0.02254345 m³, mendekati netral buoyancy) |
| **Dimensi** | 540mm × 280mm × 240mm |

---

## 🧮 Model Matematis Utama

Digital Twin ini mengimplementasikan model fisika berikut dari Fossen (2021):

### Persamaan Gerak 6-DOF Fossen
```
M ν̇ + C(ν)ν + D(ν_r)ν_r + g(η) = τ_thruster + τ_env + τ_pinn
```

### Matriks Massa (M = M_RB + M_A)

| DOF | M_RB | M_A (Added Mass) | **M_total** | **M⁻¹** |
|:----|:-----|:-----------------|:------------|:---------|
| Surge | 22.5 kg | 5.5 kg | **28.0 kg** | 0.03571 |
| Sway | 22.5 kg | 12.7 kg | **35.2 kg** | 0.02841 |
| Heave | 22.5 kg | 14.6 kg | **37.1 kg** | 0.02695 |
| Roll | 0.12 kg·m² | 0.12 kg·m² | **0.24 kg·m²** | 4.16667 |
| Pitch | 0.22 kg·m² | 0.12 kg·m² | **0.34 kg·m²** | 2.94118 |
| Yaw | 0.24 kg·m² | 0.12 kg·m² | **0.36 kg·m²** | 2.77778 |

### Koefisien Redaman Hidrodinamis (Damping)

| DOF | Linear (D_L) | Quadratic (D_Q) |
|:----|:-------------|:----------------|
| Surge | Xu = 4.03 N·s/m | Xuu = 18.18 N·s²/m² |
| Sway | Yv = 6.22 N·s/m | Yvv = 21.66 N·s²/m² |
| Heave | Zw = 5.18 N·s/m | Zww = 36.99 N·s²/m² |
| Roll/Pitch/Yaw | 0.07 N·m·s/rad | 1.55 N·m·s²/rad² |

### Gaya Pemulih Hidrostatis (Restoring Forces)
```
Berat:      W = 22.5 × 9.80665 = 220.65 N
Buoyancy:   B = 996.78 × 9.80665 × 0.02254345 = 220.36 N   (kolam 26°C)
Gaya Neto:  ΔF = -0.29 N  (mendekati netral buoyancy)

Momen Pemulih (CB 25mm di atas CG):
  Roll:  K_g = +2.81 × sin(φ) N·m
  Pitch: M_g = +2.81 × sin(θ) N·m
```

### Matriks Alokasi Thruster / Thruster Allocation Matrix (6×6)
```
           T1       T2       T3       T4       T5       T6
TAM = [  +0.707,  +0.707,  +0.707,  +0.707,   0.000,   0.000 ]  ← Surge (X)
      [  -0.707,  +0.707,  +0.707,  -0.707,   0.000,   0.000 ]  ← Sway  (Y)
      [   0.000,   0.000,   0.000,   0.000,  -1.000,  -1.000 ]  ← Heave (Z)
      [   0.000,   0.000,   0.000,   0.000,   0.000,   0.000 ]  ← Roll  (K)
      [   0.000,   0.000,   0.000,   0.000,  +0.180,  -0.180 ]  ← Pitch (M)
      [  -0.177,  +0.177,  -0.177,  +0.177,   0.000,   0.000 ]  ← Yaw   (N)
```

> 📘 **Derivasi matematis lengkap dengan perhitungan numerik langkah-demi-langkah tersedia di [`THESIS_AND_TECHNICAL_ARCHITECTURE_GUIDE.md`](THESIS_AND_TECHNICAL_ARCHITECTURE_GUIDE.md)**

---

## 🧠 Engine Inti Digital Twin

| Engine | Deskripsi | File |
|:-------|:-----------|:-----|
| **Hidrodinamika Fossen 6-DOF** | Matriks nonlinear penuh M, C, D, g dengan integrasi RK4 | `src/dt-core/HydrodynamicsEngine.js` |
| **Konfigurasi Kendaraan** | Satu sumber kebenaran untuk 50+ parameter fisik | `src/dt-core/VehicleConfig.js` |
| **EKF 15-State** | Fusi sensor multi-rate (IMU 100Hz, Depth 20Hz, DVL 10Hz) dengan gating outlier Mahalanobis | `src/dt-core/StateEstimator.js` |
| **Identifikasi Sistem Box-Jenkins** | RLS pseudo-linear online (λ=0.985) mengadaptasi 6 parameter plant/noise | `src/services/SystemIdentificationEngine.js` |
| **Competition Safety Supervisor** | Latch arah, active repulsion dinding/lantai, emergency surface, fail-safe sensor | `src/services/SafetySupervisor.js` |
| **PID Flight Controller** | Depth hold, heading lock, stabilisasi attitude dengan anti-windup | `src/services/AUVMotionController.js` |
| **Dinamika Thruster** | Lookup table T200, motor lag (τ=0.35s), voltage sag, PWM deadband, clamp misi 1200–1800 µs (bench in-air: 1300–1700) | `src/services/ThrusterDynamicsModel.js` |
| **HIL Single-Thruster Twin** | Model PWM→RPM→thrust T200 + RLS adaptif; state RPM di-*fusion* (nudge gain 0.35) ke RPM asli dari sensor Hall/back-EMF via Web Serial, sehingga beban nyata (mis. hambatan air) ikut tercermin, bukan hanya prediksi open-loop | `src/dt-core/ThrusterTwinEngine.js` |
| **PINN Residual** | Physics-Informed Neural Network untuk kompensasi dinamika yang tak termodelkan | `src/dt-core/PINNResidual.js` |
| **Model Lingkungan** | Densitas air UNESCO, turbulensi arus Gauss-Markov, tekanan kedalaman | `src/dt-core/EnvironmentModel.js` |
| **Kinematika** | Integrasi kuaternion, transform Euler↔kuaternion, body↔world | `src/dt-core/Kinematics.js` |

---

## 📡 Topik ROS 2

| Topik | Tipe | Frekuensi | Deskripsi |
| :--- | :--- | :--- | :--- |
| `/odom` | `nav_msgs/Odometry` | 50 Hz | Posisi 3D, kuaternion orientasi, kecepatan |
| `/mavros/imu/data` | `sensor_msgs/Imu` | 100 Hz | Roll, pitch, yaw & akselerasi 3-axis (langsung dari Pixhawk via MAVROS) |
| `/depth` | `sensor_msgs/FluidPressure` | 20 Hz | Pengukuran kedalaman barometer MS5837 |
| `/dvl/range` | `sensor_msgs/Range` | 10 Hz | Altitude akustik DVL (jarak ke dasar) |
| `/sonar/front/range` | `sensor_msgs/Range` | ≥20 Hz | Jarak tabrakan depan |
| `/sonar/rear/range` | `sensor_msgs/Range` | ≥20 Hz | Jarak tabrakan belakang |
| `/sonar/left/range` | `sensor_msgs/Range` | ≥20 Hz | Jarak tabrakan sisi kiri |
| `/sonar/right/range` | `sensor_msgs/Range` | ≥20 Hz | Jarak tabrakan sisi kanan |
| `/gripper/command` | `std_msgs/String` | Event | Perintah gripper: `OPEN`, `CLOSE`, `GRASP`, atau `RELEASE` |
| `/battery_state` | `sensor_msgs/BatteryState` | 1 Hz | Voltage, current, sisa kapasitas 4S LiPo |
| `/cmd_vel` | `geometry_msgs/Twist` | 50 Hz | Perintah kecepatan 4-DOF (surge, sway, heave, yaw) |
| `/thruster_commands` | `std_msgs/Float64MultiArray` | 50 Hz | Output PWM/effort 6-channel thruster |
| `/thruster_outputs` | `std_msgs/Float64MultiArray` | 50 Hz | Feedback RPM thruster real-time |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | 30 Hz | Stream video kamera depan subsea |
| `/mission_state` | `std_msgs/String` | Event | Event misi nyata (flare kena, payload dijatuhkan) disinkronkan balik ke Digital Twin |
| `/mavros/state` | `mavros_msgs/State` | 1 Hz | Feedback armed/flight-mode dari kendaraan nyata (sinkronisasi fisik -> digital) |
| `/mavros/cmd/arming`, `/mavros/set_mode` | `mavros_msgs/CommandBool`, `mavros_msgs/SetMode` | Service | Tombol arm/disarm & flight-mode di dashboard memanggil service ini langsung saat terhubung live |

### Kontrol Dua Arah (Digital ↔ Fisik)

- **Posisi/odometri & baterai**: `/odom` dan `/battery_state` TIDAK punya publisher di kendaraan asli secara default (MAVROS hanya expose `/mavros/local_position/pose` + `/velocity_local` + `/mavros/battery`) — wajib jalankan `backend/sauvc26_code/odom_bridge.py` (bridge telemetry read-only, aman jalan bareng node apa pun) supaya posisi 3D di dashboard benar-benar mengikuti hull fisik (termasuk kalau didorong manual, bukan cuma digerakkan thruster), dan supaya safety-check baterai kritis di `collision_safety.py` benar-benar melihat voltage nyata.
- **Misi otonom** (`final.py`, `qualification.py`) berjalan di Jetson dan menggerakkan kendaraan secara independen lewat `/mavros/setpoint_raw/local`; keduanya melaporkan event kembali lewat `/mission_state`.
- **Manual pilot / gripper** dari dashboard dijembatani ke Pixhawk asli oleh `backend/sauvc26_code/manual_bridge.py`, yang subscribe ke `/cmd_vel` + `/gripper/command` lalu mempublish ulang setpoint MAVROS/panggilan servo. Jalankan node ini **sebagai pengganti** node misi otonom - `backend/sauvc26_code/control_lock.py` menegakkan ini secara otomatis (file-lock PID): node kedua yang mencoba start akan gagal dengan error jelas selama node kontrol pertama masih hidup, bukan cuma disiplin operator.
- Tombol arm/disarm dan flight-mode memanggil service `/mavros/cmd/arming` dan `/mavros/set_mode` yang sebenarnya (via `TopicPublisher.armDisarm`/`setFlightMode`) setiap kali dashboard terhubung live (bukan demo/SITL).

### Keselamatan Tabrakan Fisik

State machine keselamatan adalah `NORMAL -> CAUTION -> LOCKED -> EMERGENCY_SURFACE`. Empat stream sonar arah menggunakan hysteresis: wall lock aktif pada `0.55 m` dan hanya lepas di atas `1.20 m`. Arah yang terkunci mengabaikan throttle pilot yang berlanjut dan memerintahkan thrust melarikan diri secara aktif menjauhi dinding. Data basi (stale) yang lebih tua dari `0.50 s` fail-safe.

DVL ke bawah mengaktifkan floor lock pada `0.37 m` dan melepas di atas `0.70 m`. Ia membersihkan motor lag/RPM pada T5-T6 dan memerintahkan thrust melarikan diri ke atas, sehingga menahan Dive tidak akan terus membebani lantai kolam. Kebocoran hull atau baterai kritis (`<= 8%` atau `<= 13.0 V`) mengunci emergency surface. Akselerasi IMU di atas `18 m/s²` atau angular rate di atas `2.5 rad/s` memicu emergency stop di semua sumbu.

IMU tidak dapat mengukur jarak ke dinding atau lantai. IMU digunakan untuk attitude, deteksi gerakan, dan fallback benturan; pencegahan tabrakan bergantung pada sensor sonar/DVL waterproof yang terpasang dan terkalibrasi dengan benar. Uji bangku (bench-test) setiap arah topik dan jarak henti pada thrust rendah sebelum beroperasi dekat dinding atau lantai kolam.

### Persepsi & Interaksi Kompetisi

- Radar sonar menggunakan data jarak/bearing dinding dan objek arena yang sebenarnya. Kontak berkedip hanya ketika sapuan berputar melintasi bearing-nya.
- Ember merah signal-only dikecualikan dari sonar akustik dan diberi label `CV ONLY`; pipeline kamera/YOLO onboard mendeteksinya karena kendaraan ini tidak punya penerima sinyal.
- Model tabrakan menggunakan footprint hull berorientasi heading `0.58 m x 0.32 m` ditambah overlap vertikal. Flare jatuh hanya setelah kontak geometris nyata, tidak pernah dari radius waypoint yang longgar.
- IMU, kamera depan yang diturunkan, housing sonar, dan gripper vertikal dipasang di centerline bawah. Bubble jet menunjukkan arah exhaust tiap T200, termasuk reverse dan thrust vertikal.
- Pemetaan kontroler: `Circle/B` membuka atau menutup gripper; `Cross/A` melepas atau menggenggam bola. Pelepasan masuk ke drum target hanya dalam `0.55 m`; jika tidak, bola jatuh dari posisi gripper saat ini.

### Profil Rendering Real-Time

Loop kontrol, tabrakan, IMU, dan EKF tetap pada `20 Hz`. Visualisasi sonar berjalan pada `10 Hz`, telemetri turunan validasi/OOD/health pada `5 Hz`, dan viewport computer-vision sekunder pada `15 FPS`. Komponen Three.js membaca telemetri live di dalam `useFrame` untuk menghindari pembangunan ulang scene graph di setiap paket ROS.

---

## 🚀 Quick Start

### Prasyarat
- **Node.js** 18+ dan **npm**
- **Windows 10/11** (untuk aplikasi desktop native via Electron)
- **WSL2 + Ubuntu 22.04** (opsional, untuk simulasi Gazebo)

### 1. Install Dependensi
```bash
npm install
```

### 2. Jalankan Aplikasi Desktop Native (Direkomendasikan)
Klik dua kali `Launch_Desktop_App.bat` atau:
```bash
npm run desktop
```

### 3. Jalankan di Browser (Mode Development)
```bash
npm run dev
```
Buka `http://localhost:5173` di browser.

### 4. Jalankan Simulasi 3D Gazebo di WSL2
Klik dua kali `Launch_Gazebo_WSL.bat` atau di dalam WSL:
```bash
cd simulator
./setup_wsl_gazebo.sh
```

---

## 📁 Struktur Proyek

```
digitaltwin/
├── frontend/                       ← UI React/Vite (index.html, vite.config.js, tsconfig.json)
│   ├── src/
│   ├── dt-core/                    ← Inti engine fisika (18 modul)
│   │   ├── VehicleConfig.js        ← Semua 50+ parameter kendaraan
│   │   ├── HydrodynamicsEngine.js  ← Dinamika Fossen 6-DOF + RK4
│   │   ├── Kinematics.js           ← Transformasi koordinat
│   │   ├── EnvironmentModel.js     ← Densitas air, arus
│   │   ├── StateEstimator.js       ← EKF 15-state
│   │   └── ...
│   ├── services/                   ← Controller & bridge (12 modul)
│   │   ├── AUVMotionController.js  ← PID flight controller
│   │   ├── SystemIdentificationEngine.js ← Box-Jenkins/RLS
│   │   ├── ThrusterDynamicsModel.js ← Model motor T200
│   │   ├── MockRosConnection.js    ← Simulator ROS2 SITL
│   │   └── ...
│   └── components/                 ← Komponen UI React
│       ├── 3d/                     ← Viewport 3D Three.js
│       ├── dashboard/              ← Panel telemetri
│       └── thruster-test/          ← Halaman tes HIL T200
│   └── public/                     ← Aset statis & model 3D
├── desktop/                        ← Wrapper desktop Electron
├── backend/                        ← Kode kendaraan Jetson/ROS2 (sauvc26_code)
├── simulator/                      ← Simulasi Gazebo
└── tests/                          ← Unit & integration test
```

---

## 🛠️ Tech Stack

| Layer | Teknologi |
|:------|:------------|
| **Desktop Runtime** | Electron 44, Node.js |
| **Frontend** | React 19, Vite, Vanilla CSS Design System |
| **Grafis 3D** | Three.js, React Three Fiber, Drei, WebGL |
| **Engine Fisika** | Hidrodinamika Fossen 6-DOF kustom, Integrator RK4 |
| **System ID** | ARMAX / ARX / OE / Box-Jenkins, Recursive Least Squares (λ=0.985) |
| **State Estimation** | Extended Kalman Filter 15-State, Mahalanobis Gating |
| **Robotika** | ROS 2 Humble / Jazzy, roslibjs, rosbridge_server |
| **Simulasi** | Gazebo Classic (WSLg), UUV Simulator Plugins |
| **Hardware** | NVIDIA Jetson Orin Nano, BlueRobotics T200, MS5837, Pixhawk 2.4.8 |

---

## 📚 Referensi

1. T. I. Fossen, *"Handbook of Marine Craft Hydrodynamics and Motion Control"*, 2nd ed., Wiley, 2021.
2. Berg, V., *"Development and Commissioning of a DP System for ROV SF 30k"*, NTNU, 2012.
3. Wu, G., *"Identification of Hydrodynamic Coefficients for an ROV"*, J. Ocean Engineering, 2018.
4. Blue Robotics, *"T200 Thruster Performance Data"*, bluerobotics.com, 2019.
5. UNESCO, *"International Equation of State of Seawater"*, Tech. Papers No. 36, 1980.

---

Dikembangkan dengan ❤️ untuk **Tim Amarine FILKOM Universitas Brawijaya**.
