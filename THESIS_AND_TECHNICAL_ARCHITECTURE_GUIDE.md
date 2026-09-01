# 📘 Panduan Teknis & Arsitektur Digital Twin AUV 6-DOF
## Untuk Presentasi, Sidang Tugas Akhir / Skripsi, dan Dokumentasi Teknis

Dokumen ini berisi penjelasan komprehensif mengenai arsitektur sistem, pemodelan matematika, matriks alokasi thruster, identifikasi sistem (*System Identification*), adaptasi online RLS (*Recursive Least Squares*), serta integrasi hardware ROS2 untuk robot **Autonomous Underwater Vehicle (AUV)**.

---

## 1. 🏗️ Arsitektur Sistem Digital Twin (HIL & SITL)

Digital Twin ini menghubungkan komputasi fisik (kapal nyata / Jetson Nano) dengan lingkungan simulasi 3D real-time berkinerja tinggi:

```
┌───────────────────────────────┐               ┌────────────────────────────────────────┐
│     ROBOT NYATA / SIMULATOR   │               │       DIGITAL TWIN DESKTOP APP         │
│  (Jetson Nano / WSL2 Gazebo)  │               │        (Electron + React Three.js)     │
├───────────────────────────────┤               ├────────────────────────────────────────┤
│ • ROS 2 Humble / Jazzy Nodes  │  WebSocket    │ • 3D CAD Viewport (RTX 4050 GPU)       │
│ • Sensor IMU, Depth, DVL      │ ────────────> │ • Telemetri 6-DOF Pose & Heading       │
│ • Subsea Camera Feed          │  (Port 9090)  │ • ARMAX / RLS SysID Adaptive Engine    │
│ • ESC / PWM Thruster Driver   │ <──────────── │ • PID Closed-Loop Motion Controller    │
│ • Matriks Alokasi Thruster    │   /cmd_vel    │ • YOLO Subsea Vision Frustum Overlay   │
└───────────────────────────────┘               └────────────────────────────────────────┘
```

---

## 2. 🌀 Matriks Alokasi Thruster (Thruster Allocation Matrix - TAM)

Kapal AUV kamu menggunakan konfigurasi **6 Unit BlueRobotics T200 Thrusters**:
- **4x Thruster Horizontal di 4 Sudut (Miring 45°)**: Mengatur gerak Maju/Mundur (*Surge*), Geser Samping (*Sway*), dan Putar Haluan (*Yaw*).
- **2x Thruster Vertikal di Bodi Kuning Depan & Belakang**: Mengatur gerak Menyelam/Naik (*Heave*) dan Kemiringan (*Pitch*).

### Pemetaan Matematika ($B \in \mathbb{R}^{6 \times 4}$):

$$\begin{bmatrix} T_1 \\ T_2 \\ T_3 \\ T_4 \\ T_5 \\ T_6 \end{bmatrix} = \begin{bmatrix} 
\cos(45^\circ) & -\sin(45^\circ) & 0 & L_1 \\
\cos(45^\circ) & \sin(45^\circ) & 0 & -L_1 \\
\cos(45^\circ) & \sin(45^\circ) & 0 & L_2 \\
\cos(45^\circ) & -\sin(45^\circ) & 0 & -L_2 \\
0 & 0 & 1 & L_v \\
0 & 0 & 1 & -L_v
\end{bmatrix}^\dagger \begin{bmatrix} F_{\text{surge}} \\ F_{\text{sway}} \\ F_{\text{heave}} \\ M_{\text{yaw}} \end{bmatrix}$$

- **Maju (*Surge Forward*)**: $T_1 > 0, T_2 > 0, T_3 > 0, T_4 > 0$ (4 thruster sudut aktif, thruster vertikal diam).
- **Mundur (*Surge Backward*)**: $T_1 < 0, T_2 < 0, T_3 < 0, T_4 < 0$.
- **Geser Kanan (*Sway Right*)**: $T_1 < 0, T_2 > 0, T_3 > 0, T_4 < 0$.
- **Geser Kiri (*Sway Left*)**: $T_1 > 0, T_2 < 0, T_3 < 0, T_4 > 0$.
- **Putar Haluan (*Yaw Turning*)**: Sisi kiri dorong maju, sisi kanan dorong mundur.
- **Selam Turun (*Dive Down*)**: $T_5 > 0, T_6 > 0$ (2 thruster kuning mendorong air ke atas).
- **Naik ke Permukaan (*Surface Up*)**: $T_5 < 0, T_6 < 0$ (2 thruster kuning mendorong air ke bawah).

---

## 3. 🌊 Fisika Gaya Apung Alami (*Positive Buoyancy / Fail-Safe*)

- **Prinsip Archimedes**: Gaya apung total fluida dirancang sedikit lebih besar dari berat total kapal:
  $$F_b = \rho \cdot g \cdot V_{\text{displaced}} > W = m \cdot g \quad (+200\text{g} \text{ s.d. } +500\text{g})$$
- **Saat Mode IDLE / DISARM**:
  Thruster vertikal $T_5 = 0, T_6 = 0$. Kapal secara otomatis dan aman melayang naik ke permukaan (`depth ~ 0.15m`), mencegah kapal tenggelam atau hilang saat kehilangan daya.
- **Saat Mode ALT_HOLD / STABILIZE**:
  Pengendali PID kedalaman secara aktif memerintahkan $T_5$ & $T_6$ untuk melawan gaya apung dan menjaga kedalaman stabil di target (misal $0.8\text{m}$).

---

## 4. 🧠 Identifikasi Sistem (SysID) & Adaptasi Online RLS

### Model Polinomial Diskrit ARMAX:
$$A(q^{-1}) y(t) = B(q^{-1}) u(t - d) + C(q^{-1}) e(t)$$

- $y(t)$: Respons kecepatan / posisi AUV pada waktu diskrit $t$.
- $u(t)$: Input gaya dorong thruster (PWM T200 pada 16V).
- $e(t)$: Noise / gangguan hidrodinamika air.

### Algoritma Recursive Least Squares (RLS) dengan Forgetting Factor:
Untuk mengadaptasi parameter model ketika ada perubahan arus air, beban baterai, atau densitas kolam secara *real-time*:

1. **Vektor Regresor**: $\varphi(t) = [-y(t-1), -y(t-2), u(t-1), u(t-2)]^T$
2. **Gain Adaptif**: $K(t) = \frac{P(t-1)\varphi(t)}{\lambda + \varphi^T(t)P(t-1)\varphi(t)}$
3. **Koreksi Parameter**: $\hat{\theta}(t) = \hat{\theta}(t-1) + K(t) [y(t) - \varphi^T(t)\hat{\theta}(t-1)]$
4. **Pembaruan Kovarians**: $P(t) = \frac{1}{\lambda} \left[ I - K(t)\varphi^T(t) \right] P(t-1)$
*(Dengan forgetting factor $\lambda = 0.985$ untuk pelacakan parameter yang cepat dan stabil).*

---

## 5. 📡 Topik ROS 2 yang Terhubung

| Topik ROS 2 | Tipe Pesan | Deskripsi |
| :--- | :--- | :--- |
| `/odom` | `nav_msgs/msg/Odometry` | Posisi 3D, orientasi kuaternion, dan kecepatan linear/angular AUV |
| `/imu/data` | `sensor_msgs/msg/Imu` | Orientasi sudut (Roll, Pitch, Yaw) dan akselerasi linier 3-sumbu |
| `/depth` | `sensor_msgs/msg/FluidPressure` | Tekanan air & kedalaman dari sensor Barometer MS5837 |
| `/dvl/range` | `sensor_msgs/msg/Range` | Jarak altimeter akustik DVL ke dasar kolam |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | Tegangan 4S LiPo (16V nominal), arus beban, dan kapasitas baterai |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Perintah kecepatan 4-DOF (*Surge, Sway, Heave, Yaw*) |
| `/thruster_commands` | `std_msgs/msg/Float64MultiArray` | Perintah direct PWM / Effort untuk ke-6 unit thruster |
| `/thruster_outputs` | `std_msgs/msg/Float64MultiArray` | Umpan balik putaran nyata ke-6 thruster dari hardware Jetson |
| `/camera/image_raw/compressed`| `sensor_msgs/msg/CompressedImage` | Streaming video kamera depan beresolusi tinggi |

---

## 6. 🚀 Ringkasan Peluncuran (Launcher Summary)

1. **Aplikasi Desktop Utama (Windows Native Software)**:
   - Double-click **`Launch_Desktop_App.bat`** (Membuka jendela dashboard 3D dengan akselerasi GPU RTX 4050).
2. **Simulasi ROS 2 Docker Desktop**:
   - Double-click **`simulator/start_simulation.bat`** (Menjalankan simulator backend ROS 2 Humble di background).
3. **Simulasi 3D Gazebo di WSL2**:
   - Double-click **`Launch_Gazebo_WSL.bat`** (Membuka jendela 3D Gazebo native via WSLg).
