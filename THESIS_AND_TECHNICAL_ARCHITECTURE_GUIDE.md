# 📘 Buku Panduan Teknis & Formulasi Lengkap Matematika Digital Twin AUV 6-DOF
##  Dokumentasi Arsitektur
**Autonomous Underwater Vehicle (AUV) Amarine — FILKOM Universitas Brawijaya**

---

## 📑 Daftar Isi
1. [Arsitektur Sistem & Aliran Data Digital Twin (HIL & SITL)](#1-arsitektur-sistem--aliran-data-digital-twin-hil--sitl)
2. [Sistem Koordinat & Kinematika 6-DOF](#2-sistem-koordinat--kinematika-6-dof)
3. [Perhitungan Lengkap Dinamika Hidrodinamika 6-DOF (Persamaan Fossen)](#3-perhitungan-lengkap-dinamika-hidrodinamika-6-dof-persamaan-fossen)
4. [Perhitungan Matriks Massa Total 6x6 (M = M_RB + M_A) & Inversnya](#4-perhitungan-matriks-massa-total-6x6-m--m_rb--m_a--inversnya)
5. [Perhitungan Matriks Coriolis & Sentripetal 6x6 (C(v))](#5-perhitungan-matriks-coriolis--sentripetal-6x6-cv)
6. [Perhitungan Matriks Redaman Gesekan Air Nonlinier (D_L + D_Q|v|)](#6-perhitungan-matriks-redaman-gesekan-air-nonlinier-d_l--d_qv)
7. [Perhitungan Hidrostatis, Gaya Apung Archimedes & Momen Penegak g(eta)](#7-perhitungan-hidrostatis-gaya-apung-archimedes--momen-penegak-geta)
8. [Perhitungan Matriks Alokasi Thruster (TAM 6x6) & Inversi Pseudo-Inverse](#8-perhitungan-matriks-alokasi-thruster-tam-6x6--inversi-pseudo-inverse)
9. [Perhitungan Fusi Sensor Multi-Rate: 15-State Extended Kalman Filter (EKF)](#9-perhitungan-fusi-sensor-multi-rate-15-state-extended-kalman-filter-ekf)
10. [Perhitungan Identifikasi Sistem Daring ARMAX & Adaptasi RLS](#10-perhitungan-identifikasi-sistem-daring-armax--adaptasi-rls)
11. [Perhitungan Pengendali Closed-Loop PID + Kompensasi Feedforward](#11-perhitungan-pengendali-closed-loop-pid--kompensasi-feedforward)
12. [Perhitungan Integrasi Numerik Runge-Kutta Orde ke-4 (RK4)](#12-perhitungan-integrasi-numerik-runge-kutta-orde-ke-4-rk4)
13. [Tabel Komprehensif Seluruh Parameter & Satuan SI Terkalibrasi](#13-tabel-komprehensif-seluruh-parameter--satuan-si-terkalibrasi)
14. [Panduan Menjalankan Sistem](#14-panduan-menjalankan-sistem)

---

## 1. 🏗️ Arsitektur Sistem & Aliran Data Digital Twin (HIL & SITL)

Digital Twin ini menghubungkan wahana fisik nyata (*Physical Twin*) dengan lingkungan simulasi virtual berkinerja tinggi (*Virtual Twin*) secara dua arah (*bi-directional real-time telemetry*):

```
┌─────────────────────────────────────────────────────────────┐
│                 PHYSICAL TWIN (HARDWARE / HIL)              │
│  - Komputer Onboard: NVIDIA Jetson Nano                     │
│  - Sensor: IMU 9-DOF, Barometer MS5837, DVL, Forward Cam   │
│  - Aktuator: 6x BlueRobotics T200 Brushless ESC             │
└──────────────────────────────┬──────────────────────────────┘
                               │
            Telemetri ROS 2    │  WebSocket JSON / Protobuf
             /odom, /imu,      │  (Port 9090 / rosbridge_server)
             /depth, /dvl      │  Frekuensi: 20 Hz - 100 Hz
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                 VIRTUAL TWIN (DIGITAL TWIN ENGINE)          │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ 1. 6-DOF Fossen Hydrodynamics Physics Engine (RK4)     │  │
│  │ 2. 15-State Multi-Rate Extended Kalman Filter (EKF)   │  │
│  │ 3. Online RLS System Identification (ARMAX Model)     │  │
│  │ 4. Closed-Loop PID + TAM Thruster Allocator           │  │
│  │ 5. PINN (Physics-Informed Neural Network) Residual    │  │
│  │ 6. Deteksi Outlier Sensor Mahalanobis Gating          │  │
│  └───────────────────────────────────────────────────────┘  │
│                               │                             │
│                               ▼                             │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ 3D Rendering CAD Viewport (Three.js WebGL / RTX 4050) │  │
│  │ - 6-DOF Pose Sync, Thruster Vector Visualizer         │  │
│  │ - Live Sensor Uncertainty Ellipsoid & Particle Trails │  │
│  └───────────────────────────────────────────────────────┘  │
└──────────────────────────────┬──────────────────────────────┘
                               │ Perintah Kontrol: /cmd_vel
                               ▼
┌─────────────────────────────────────────────────────────────┐
│            SITL SIMULATOR (Gazebo Garden / ROS 2 Humble)    │
│  - WSL2 Ubuntu 22.04 LTS (NVIDIA Container Toolkit)         │
│  - UUV Simulator / Buoyancy & Hydrodynamics Plugins         │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 🌐 Sistem Koordinat & Kinematika 6-DOF

### A. Definisi Kerangka Acuan (Reference Frames):
1. **Kerangka Acuan Inersia / Bumi {n} (NED - North East Down)**:
   - Sumbu **x_n**: Arah Utara (*North*) [meter]
   - Sumbu **y_n**: Arah Timur (*East*) [meter]
   - Sumbu **z_n**: Arah Bawah / Kedalaman (*Down*) [meter]
2. **Kerangka Acuan Bodi {b} (Body-Fixed Frame)**:
   - Sumbu **x_b**: Sumbu memanjang kapal, positif ke arah moncong depan (*Surge*)
   - Sumbu **y_b**: Sumbu melintang kapal, positif ke arah lambung kanan (*Sway*)
   - Sumbu **z_b**: Sumbu tegak kapal, positif ke arah lunas bawah (*Heave*)

---

### B. Vektor Keadaan (State Vectors):

- **Vektor Posisi & Orientasi Euler dalam Kerangka NED {n}**:
  `eta = [x, y, z, phi, theta, psi]^T`
  - x = posisi utara (m), y = posisi timur (m), z = kedalaman (m)
  - phi = sudut roll (rad), theta = sudut pitch (rad), psi = sudut yaw/haluan (rad)

- **Vektor Kecepatan Linier & Kecepatan Sudut dalam Kerangka Bodi {b}**:
  `nu = [u, v, w, p, q, r]^T`
  - u = kecepatan maju (m/s), v = kecepatan geser samping (m/s), w = kecepatan selam (m/s)
  - p = laju putar roll (rad/s), q = laju putar pitch (rad/s), r = laju putar yaw (rad/s)

- **Vektor Gaya & Momen Generalisasi dalam Kerangka Bodi {b}**:
  `tau = [X, Y, Z, K, M, N]^T`
  - X = gaya maju (N), Y = gaya geser samping (N), Z = gaya selam (N)
  - K = momen roll (N·m), M = momen pitch (N·m), N = momen yaw (N·m)

---

### C. Persamaan Transformasi Kinematika Euler:

Turunan posisi bumi `eta_dot` dihitung dari kecepatan bodi `nu`:
`[x_dot, y_dot, z_dot]^T = R_b_to_n * [u, v, w]^T`
`[phi_dot, theta_dot, psi_dot]^T = T_Theta * [p, q, r]^T`

#### 1. Matriks Rotasi Linier R_b_to_n (3x3):
```
R_b_to_n = [
  [ cos(psi)*cos(theta),  -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi),   sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi) ],
  [ sin(psi)*cos(theta),   cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi),  -cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi) ],
  [ -sin(theta),           cos(theta)*sin(phi),                                cos(theta)*cos(phi)                              ]
]
```

#### 2. Matriks Transformasi Kecepatan Sudut T_Theta (3x3):
```
T_Theta = [
  [ 1,  sin(phi)*tan(theta),  cos(phi)*tan(theta) ],
  [ 0,  cos(phi),            -sin(phi)            ],
  [ 0,  sin(phi)/cos(theta),  cos(phi)/cos(theta) ]
]
```

#### 3. Kinematika Unit Kuaternion (Non-Singular):
Untuk mencegah *gimbal lock* saat theta mendekati 90 derajat, digunakan kuaternion `q = [qw, qx, qy, qz]^T`:
`q_dot = 0.5 * Omega * q`

Di mana matriks Omega adalah:
```
Omega = [
  [  0, -p, -q, -r ],
  [  p,  0,  r, -q ],
  [  q, -r,  0,  p ],
  [  r,  q, -p,  0 ]
]
```

---

## 3. 🌊 Perhitungan Lengkap Dinamika Hidrodinamika 6-DOF (Persamaan Fossen)

Persamaan gerak dinamika maritim nonlinier Fossen:

`M * nu_dot + C(nu) * nu + D(nu_r) * nu_r + g(eta) = tau_thruster + tau_env + tau_pinn`

Di mana:
- `M`: Matriks Massa Inersia Total (6x6) = Massa Bodi Kaku (M_RB) + Massa Tambah Fluida (M_A)
- `C(nu)`: Matriks Gaya Coriolis & Sentripetal (6x6)
- `D(nu_r)`: Matriks Redaman Hidrodinamika Fluida (6x6)
- `g(eta)`: Vektor Gaya & Momen Pemulih Hidrostatis (6x1)
- `nu_r = nu - nu_current`: Kecepatan relatif terhadap arus air
- `tau_thruster`: Gaya dorong total dari 6 unit motor
- `tau_env`: Gangguan gaya lingkungan (gelombang air)
- `tau_pinn`: Kompensasi residual neural network AI

---

## 4. 🧮 Perhitungan Matriks Massa Total 6x6 (M = M_RB + M_A) & Inversnya

### A. Matriks Massa Bodi Kaku (M_RB):
Diketahui parameter robot:
- Massa kering kapal: `m = 11.5 kg`
- Pusat gravitasi: `CG = [xG, yG, zG] = [0.0, 0.0, 0.0] m`
- Momen Inersia Roll: `Ixx = 0.12 kg·m²`
- Momen Inersia Pitch: `Iyy = 0.22 kg·m²`
- Momen Inersia Yaw: `Izz = 0.24 kg·m²`
- Produk Inersia: `Ixy = Ixz = Iyz = 0.0`

```
M_RB = [
  [ 11.5,  0.0,   0.0,   0.0,   0.0,   0.0  ],
  [  0.0, 11.5,   0.0,   0.0,   0.0,   0.0  ],
  [  0.0,  0.0,  11.5,   0.0,   0.0,   0.0  ],
  [  0.0,  0.0,   0.0,   0.12,  0.0,   0.0  ],
  [  0.0,  0.0,   0.0,   0.0,   0.22,  0.0  ],
  [  0.0,  0.0,   0.0,   0.0,   0.0,   0.24 ]
]
```

---

### B. Matriks Massa Tambah Fluida (Added Mass - M_A):
Air di sekitar lambung yang ikut terdorong memiliki inersia terkalibrasi:
- Added mass Surge: `Xu_dot = 5.5 kg`
- Added mass Sway: `Yv_dot = 12.7 kg` (badan samping lebar)
- Added mass Heave: `Zw_dot = 14.6 kg` (pelat atas/bawah datar)
- Added inertia Roll: `Kp_dot = 0.12 kg·m²`
- Added inertia Pitch: `Mq_dot = 0.12 kg·m²`
- Added inertia Yaw: `Nr_dot = 0.12 kg·m²`

```
M_A = [
  [ 5.5,   0.0,   0.0,   0.0,   0.0,   0.0  ],
  [ 0.0,  12.7,   0.0,   0.0,   0.0,   0.0  ],
  [ 0.0,   0.0,  14.6,   0.0,   0.0,   0.0  ],
  [ 0.0,   0.0,   0.0,   0.12,  0.0,   0.0  ],
  [ 0.0,   0.0,   0.0,   0.0,   0.12,  0.0  ],
  [ 0.0,   0.0,   0.0,   0.0,   0.0,   0.12 ]
]
```

---

### C. Matriks Massa Inersia Efektif Total (M = M_RB + M_A):
```
M = [
  [ 17.0,  0.0,   0.0,   0.0,   0.0,   0.0  ],
  [  0.0, 24.2,   0.0,   0.0,   0.0,   0.0  ],
  [  0.0,  0.0,  26.1,   0.0,   0.0,   0.0  ],
  [  0.0,  0.0,   0.0,   0.24,  0.0,   0.0  ],
  [  0.0,  0.0,   0.0,   0.0,   0.34,  0.0  ],
  [  0.0,  0.0,   0.0,   0.0,   0.0,   0.36 ]
]
```

---

### D. Invers Matriks Massa Total (M^-1):
Invers ini digunakan untuk menghitung percepatan wahana `nu_dot = M^-1 * Sigma_Gaya`:
```
M^-1 = [
  [ 1/17.0,   0,       0,       0,      0,      0     ],
  [   0,    1/24.2,    0,       0,      0,      0     ],
  [   0,      0,     1/26.1,    0,      0,      0     ],
  [   0,      0,       0,     1/0.24,   0,      0     ],
  [   0,      0,       0,       0,    1/0.34,   0     ],
  [   0,      0,       0,       0,      0,    1/0.36  ]
]

M^-1 = [
  [ 0.05882,  0.00000,  0.00000,  0.00000,  0.00000,  0.00000 ],
  [ 0.00000,  0.04132,  0.00000,  0.00000,  0.00000,  0.00000 ],
  [ 0.00000,  0.00000,  0.03831,  0.00000,  0.00000,  0.00000 ],
  [ 0.00000,  0.00000,  0.00000,  4.16667,  0.00000,  0.00000 ],
  [ 0.00000,  0.00000,  0.00000,  0.00000,  2.94118,  0.00000 ],
  [ 0.00000,  0.00000,  0.00000,  0.00000,  0.00000,  2.77778 ]
]
```

---

## 5. 🔄 Perhitungan Matriks Coriolis & Sentripetal 6x6 (C(v))

Matriks Coriolis dimodelkan menggunakan matriks perkalian silang *skew-symmetric* `S(a)`:
```
S(a) = [
  [   0,  -a3,   a2 ],
  [  a3,    0,  -a1 ],
  [ -a2,   a1,    0 ]
]
```

Struktur lengkap matriks Coriolis `C(nu) = C_RB(nu) + C_A(nu_r)`:
```
C(nu) = [
  [ 0_{3x3},               -S(a_RB + a_A) ],
  [ -S(a_RB + a_A),        -S(b_RB + b_A) ]
]
```

Di mana vektor momentumnya adalah:
- `a_RB = m * [u, v, w] = [11.5*u, 11.5*v, 11.5*w]`
- `b_RB = [Ixx*p, Iyy*q, Izz*r] = [0.12*p, 0.22*q, 0.24*r]`
- `a_A = [Xu_dot*ur, Yv_dot*vr, Zw_dot*wr] = [5.5*ur, 12.7*vr, 14.6*wr]`
- `b_A = [Kp_dot*p, Mq_dot*q, Nr_dot*r] = [0.12*p, 0.12*q, 0.12*r]`

---

## 6. 🌊 Perhitungan Matriks Redaman Gesekan Air Nonlinier (D_L + D_Q|v|)

Gaya dan momen hambatan air dihitung dari kombinasi redaman linier (*skin friction*) dan redaman kuadratik (*vortex shedding drag*):

### Persamaan Tiap Sumbu:
1. **Hambatan Surge (Maju)**:
   `F_drag_surge = -(4.03 * u + 18.18 * |u| * u)`
2. **Hambatan Sway (Geser Samping)**:
   `F_drag_sway  = -(6.22 * v + 21.66 * |v| * v)`
3. **Hambatan Heave (Menyelam)**:
   `F_drag_heave = -(5.18 * w + 36.99 * |w| * w)`
4. **Hambatan Roll (Guling)**:
   `M_drag_roll  = -(0.07 * p + 1.55 * |p| * p)`
5. **Hambatan Pitch (Angguk)**:
   `M_drag_pitch = -(0.07 * q + 1.55 * |q| * q)`
6. **Hambatan Yaw (Belok)**:
   `M_drag_yaw   = -(0.07 * r + 1.55 * |r| * r)`

### Contoh Hitungan Nyata:
Jika kapal bergerak maju `u = 0.6 m/s` dan belok `r = 0.3 rad/s`:
- `F_drag_surge = -(4.03 * 0.6 + 18.18 * 0.36) = -(2.418 + 6.545) = -8.963 Newton`
- `M_drag_yaw   = -(0.07 * 0.3 + 1.55 * 0.09) = -(0.021 + 0.1395) = -0.1605 N·m`

---

## 7. ⚖️ Perhitungan Hidrostatis, Gaya Apung Archimedes & Momen Penegak g(eta)

### A. Perhitungan Keseimbangan Vertikal Archimedes:
1. **Berat Total Kapal (W)**:
   `W = m * g = 11.5 kg * 9.80665 m/s² = 112.7765 Newton`
2. **Gaya Apung Fluida (B)**:
   `B = rho_water * g * V_displaced = 998.2 kg/m³ * 9.80665 m/s² * 0.01225 m³ = 119.9140 Newton`
3. **Gaya Bersih ke Atas (Delta F)**:
   `Delta F = B - W = 119.9140 N - 112.7765 N = +7.1375 Newton`
   *(Setara dengan gaya angkat +727.8 gram-force).*

---

### B. Vektor Gaya & Momen Pemulih Hidrostatis g(eta):
Pusat Gravitasi `CG = [0, 0, 0] m`, Pusat Apung `CB = [0, 0, -0.025] m` (CB berada 25 mm di atas CG).

```
g(eta) = [
  (W - B) * sin(theta),
  -(W - B) * cos(theta) * sin(phi),
  -(W - B) * cos(theta) * cos(phi),
  -(yG*W - yB*B)*cos(theta)*cos(phi) + (zG*W - zB*B)*cos(theta)*sin(phi),
  (zG*W - zB*B)*sin(theta) + (xG*W - xB*B)*cos(theta)*cos(phi),
  -(xG*W - xB*B)*cos(theta)*sin(phi) - (yG*W - yB*B)*sin(theta)
]
```

Substitusi angka robot:
```
g(eta) = [
  -7.1375 * sin(theta),
  +7.1375 * cos(theta) * sin(phi),
  +7.1375 * cos(theta) * cos(phi),
  +2.9978 * cos(theta) * sin(phi),
  +2.9978 * sin(theta),
  0.0
]
```

- **Momen Penegak Roll**: `K_g = +2.9978 * sin(phi) N·m` (mengembalikan posisi datar jika kapal miring).
- **Momen Penegak Pitch**: `M_g = +2.9978 * sin(theta) N·m` (mengembalikan hidung kapal mendatar jika menukik).

---

## 8. 🌀 Perhitungan Matriks Alokasi Thruster (TAM 6x6) & Inversi Pseudo-Inverse

### A. Geometri Posisi & Vektor Arah 6 Motor Thruster:
| Motor | Posisi [x, y, z] (m) | Vektor Gaya [dx, dy, dz] | Fungsi Gerakan |
| :--- | :--- | :--- | :--- |
| **T1 (Depan-Kiri)** | [+0.14, +0.11, 0.0] | [cos(45°), -sin(45°), 0] = [+0.7071, -0.7071, 0] | Maju (+), Geser Kiri (-), Yaw Kanan (+) |
| **T2 (Depan-Kanan)**| [+0.14, -0.11, 0.0] | [cos(45°), +sin(45°), 0] = [+0.7071, +0.7071, 0] | Maju (+), Geser Kanan (+), Yaw Kiri (-) |
| **T3 (Belakang-Kiri)**| [-0.14, +0.11, 0.0] | [cos(45°), +sin(45°), 0] = [+0.7071, +0.7071, 0] | Maju (+), Geser Kanan (+), Yaw Kanan (+) |
| **T4 (Belakang-Kanan)**| [-0.14, -0.11, 0.0] | [cos(45°), -sin(45°), 0] = [+0.7071, -0.7071, 0] | Maju (+), Geser Kiri (-), Yaw Kiri (-) |
| **T5 (Vertikal Depan)**| [+0.18, 0.0, 0.0] | [0, 0, -1.0] | Menyelam Turun (-Z), Pitch Menukik (-M) |
| **T6 (Vertikal Belakang)**| [-0.18, 0.0, 0.0] | [0, 0, -1.0] | Menyelam Turun (-Z), Pitch Mendongak (+M) |

Lengan momen yaw:
`L_arm = x * sin(45°) + y * cos(45°) = 0.14 * 0.7071 + 0.11 * 0.7071 = 0.1768 meter`

---

### B. Matriks Konfigurasi Alokasi Thruster T_alloc (6x6):
`tau = T_alloc * u_T`

```
T_alloc = [
  [ +0.7071,  +0.7071,  +0.7071,  +0.7071,   0.0000,   0.0000 ],
  [ -0.7071,  +0.7071,  +0.7071,  -0.7071,   0.0000,   0.0000 ],
  [  0.0000,   0.0000,   0.0000,   0.0000,  -1.0000,  -1.0000 ],
  [  0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000 ],
  [  0.0000,   0.0000,   0.0000,   0.0000,  +0.1800,  -0.1800 ],
  [ +0.1768,  -0.1768,  +0.1768,  -0.1768,   0.0000,   0.0000 ]
]
```

---

### C. Solusi Closed-Form Inversi Alokasi Tenaga Motor:
`u_T = T_alloc_pseudo_inverse * tau_command`

Perhitungan daya tiap motor dari perintah gaya `[Fx, Fy, Fz, K, My, Mz]`:
- **T1 = (0.3536 * Fx) - (0.3536 * Fy) + (1.4141 * Mz)**
- **T2 = (0.3536 * Fx) + (0.3536 * Fy) - (1.4141 * Mz)**
- **T3 = (0.3536 * Fx) + (0.3536 * Fy) + (1.4141 * Mz)**
- **T4 = (0.3536 * Fx) - (0.3536 * Fy) - (1.4141 * Mz)**
- **T5 = (-0.5000 * Fz) + (2.7778 * My)**
- **T6 = (-0.5000 * Fz) - (2.7778 * My)**

---

## 9. 🛰️ Perhitungan Fusi Sensor Multi-Rate: 15-State Extended Kalman Filter (EKF)

### A. Vektor Keadaan EKF (x berukuran 15x1):
- `x[0..2]`: Posisi NED `[x, y, z]` (meter)
- `x[3..5]`: Kecepatan bodi `[u, v, w]` (meter/detik)
- `x[6..8]`: Sudut Euler `[phi, theta, psi]` (radian)
- `x[9..11]`: Bias drift akselerometer `[b_ax, b_ay, b_az]` (m/s²)
- `x[12..14]`: Bias drift giroskop `[b_gx, b_gy, b_gz]` (rad/s)

---

### B. Tahap Prediksi (Time-Update Step - Frekuensi 100 Hz):
Interval waktu IMU: `dt = 0.01 detik`.
Input sensor: akselerasi spesifik terukur `f_m = [ax, ay, az]` dan laju putar terukur `omega_m = [gx, gy, gz]`.

1. **Prediksi Keadaan**:
   - `posisi_dot = R_b_to_n(Theta) * v_body`
   - `kecepatan_dot = (f_m - b_a) - S(omega_m - b_g) * v_body + R_n_to_b * [0, 0, g]`
   - `sudut_dot = T_Theta(Theta) * (omega_m - b_g)`
   - `bias_a_dot = 0` (model random walk drift)
   - `bias_g_dot = 0` (model random walk drift)

2. **Propagasi Kovarians Ketidakpastian (15x15)**:
   `P_k = Phi_k * P_{k-1} * Phi_k^T + Q_k`
   Di mana `Phi_k = I_15 + F_k * dt` dengan Jacobian sistem `F_k = d(f) / d(x)`.

---

### C. Tahap Koreksi Pengukuran (Measurement-Update Step):
Saat sensor kedalaman (20 Hz) atau DVL (10 Hz) masuk:

1. **Residual Inovasi**: `y_tilde = z_meas - h(x_pred)`
2. **Kovarians Inovasi**: `S = H * P * H^T + R`
3. **Validasi Outlier Chi-Square Gating**:
   `d_M^2 = y_tilde^T * S^-1 * y_tilde <= gamma_threshold` (threshold = 9.0 untuk sensor 1D)
4. **Kalman Gain Optimal**: `K = P * H^T * S^-1`
5. **Update State & Kovarians (Bentuk Joseph)**:
   `x_updated = x_pred + K * y_tilde`
   `P_updated = (I - K*H) * P * (I - K*H)^T + K * R * K^T`

---

## 10. 🧠 Perhitungan Identifikasi Sistem Daring ARMAX & Adaptasi RLS

Model polinomial diskrit input-output:
`A(q^-1) * y(t) = B(q^-1) * u(t-d) + C(q^-1) * e(t)`

Bentuk regresi linier:
`y(t) = phi(t)^T * theta + e(t)`

- **Vektor Regresor (4x1)**:
  `phi(t) = [ -y(t-1),  -y(t-2),  u(t-1),  u(t-2) ]^T`
- **Vektor Parameter yang Diestimasi (4x1)**:
  `theta(t) = [ a1, a2, b1, b2 ]^T`

### Persamaan Komputasi RLS dengan Forgetting Factor (lambda = 0.985):
1. **Gain Adaptif K(t)**:
   `K(t) = ( P(t-1) * phi(t) ) / ( lambda + phi(t)^T * P(t-1) * phi(t) )`
2. **Error Prediksi a Priori**:
   `epsilon(t) = y(t) - phi(t)^T * theta(t-1)`
3. **Pembaruan Parameter**:
   `theta(t) = theta(t-1) + K(t) * epsilon(t)`
4. **Pembaruan Matriks Kovarians P(t)**:
   `P(t) = (1 / lambda) * ( I_4 - K(t) * phi(t)^T ) * P(t-1)`

---

## 11. 🎯 Perhitungan Pengendali Closed-Loop PID + Kompensasi Feedforward

Persamaan umum pengendali PID:
`Output = Feedforward + Kp * Error + Ki * Integral(Error) + Kd * Derivatif(Error)`

### A. Pengendali Kedalaman (Heave Axis):
Karena ada gaya apung bersih ke atas sebesar `+7.14 Newton`, ditambahkan kompensasi feedforward `F_FF = -7.14 Newton`:

`Fz_cmd = -7.1375 + 45.0 * (z_target - z) + 8.0 * Integral(z_target - z) - 24.0 * w`

### B. Matriks Gain Pengendali PID Terkalibrasi:
| Derajat Kebebasan | Kp | Ki | Kd | Batas Output Maksimal |
| :--- | :--- | :--- | :--- | :--- |
| **Surge (Maju u)** | 25.0 N/(m/s) | 4.0 N/m | 12.0 N·s/m | ± 80.0 Newton |
| **Sway (Geser v)** | 22.0 N/(m/s) | 3.5 N/m | 10.0 N·s/m | ± 60.0 Newton |
| **Heave (Kedalaman z)**| 45.0 N/m | 8.0 N/(m·s) | 24.0 N·s/m | ± 80.0 Newton |
| **Yaw (Haluan psi)** | 18.0 N·m/rad | 2.5 N·m/(rad·s) | 9.5 N·m·s/rad | ± 20.0 N·m |

---

## 12. ⚙️ Perhitungan Integrasi Numerik Runge-Kutta Orde ke-4 (RK4)

Digital Twin menghitung percepatan wahana `nu_dot = f(eta, nu, tau)` pada setiap `dt = 0.02 detik` (50 Hz):

`f(eta, nu, tau) = M^-1 * [ tau_thruster + tau_pinn - C(nu)*nu - D(nu_r)*nu_r - g(eta) ]`

### 4 Tahapan Hitungan RK4:
1. `k1 = f(t, nu)`
2. `k2 = f(t + 0.5*dt, nu + 0.5*dt*k1)`
3. `k3 = f(t + 0.5*dt, nu + 0.5*dt*k2)`
4. `k4 = f(t + dt, nu + dt*k3)`
5. `nu_next = nu + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)`

Integrasi posisi:
`eta_next = eta + dt * J(eta) * nu_next`

---

## 13. 📋 Tabel Komprehensif Seluruh Parameter & Satuan SI Terkalibrasi

| Nama Parameter | Notasi Simbol | Nilai Numerik Terkalibrasi | Satuan SI | Keterangan & Sumber |
| :--- | :--- | :--- | :--- | :--- |
| **Massa Kering Kapal** | m | 11.5 | kg | Penimbangan digital darat |
| **Momen Inersia Roll** | Ixx | 0.12 | kg·m² | Ekstraksi CAD Mesh & URDF |
| **Momen Inersia Pitch**| Iyy | 0.22 | kg·m² | Ekstraksi CAD Mesh & URDF |
| **Momen Inersia Yaw**  | Izz | 0.24 | kg·m² | Ekstraksi CAD Mesh & URDF |
| **Volume Benaman Air** | V | 0.01225 | m³ (12.25 L) | Uji benaman Archimedes |
| **Densitas Air Kolam** | rho | 998.2 | kg/m³ | Pengukuran suhu 20°C |
| **Percepatan Gravitasi**| g | 9.80665 | m/s² | Konstanta geofisika lokal |
| **Pusat Gravitasi (CG)**| [xG, yG, zG] | [0.0, 0.0, 0.0] | meter | Asal kerangka bodi {b} |
| **Pusat Apung (CB)**   | [xB, yB, zB] | [0.0, 0.0, -0.025] | meter | Posisi busa apung atas |
| **Added Mass Surge**   | Xu_dot | 5.5 | kg | Towing tank / Berg (2012) |
| **Added Mass Sway**    | Yv_dot | 12.7 | kg | Towing tank / Wu (2018) |
| **Added Mass Heave**   | Zw_dot | 14.6 | kg | Uji selam / Fossen (2021) |
| **Damping Linier Surge**| Xu | 4.03 | N·s/m | Uji deselerasi luncur kolam |
| **Damping Quad Surge** | Xuu | 18.18 | N·s²/m² | Uji kecepatan terminal kolam |
| **Damping Linier Sway** | Yv | 6.22 | N·s/m | Uji geser menyamping |
| **Damping Quad Sway**  | Yvv | 21.66 | N·s²/m² | Uji gerak lateral kolam |
| **Damping Linier Heave**| Zw | 5.18 | N·s/m | Uji selam vertikal kolam |
| **Damping Quad Heave** | Zww | 36.99 | N·s²/m² | Uji selam vertikal kolam |
| **Gaya Dorong Maks Fwd**| T_max_fwd | +50.0 | Newton (5.1 kgf) | Datasheet T200 @ 16V |
| **Gaya Dorong Maks Rev**| T_max_rev | -40.2 | Newton (4.1 kgf) | Datasheet T200 @ 16V |
| **Konstanta Waktu Motor**| tau_m | 0.35 | detik | Step response dyno test |
| **Tegangan Baterai**   | V_bat | 16.0 | Volt | LiPo 4S 10.000 mAh |
| **Deadband PWM Motor** | PWM_deadband | 1470 s.d. 1530 | mikrodetik (μs) | BlueRobotics Basic ESC |

---

## 14. 🚀 Panduan Menjalankan Sistem

1. **Aplikasi Desktop Utama (Windows Native Digital Twin)**:
   - Double-click file: **`Launch_Desktop_App.bat`** (Membuka antarmuka 3D real-time dengan akselerasi GPU RTX 4050).
2. **Backend Simulasi ROS 2 (Docker Environment)**:
   - Double-click file: **`simulator/start_simulation.bat`**.
3. **Simulasi 3D Gazebo di WSL2 (Opsional)**:
   - Double-click file: **`Launch_Gazebo_WSL.bat`**.
