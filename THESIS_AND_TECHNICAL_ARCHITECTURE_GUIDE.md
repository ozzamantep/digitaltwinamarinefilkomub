# 📘 Buku Panduan Teknis & Formulasi Lengkap Matematika Digital Twin AUV 6-DOF
## Dokumentasi Arsitektur
**Autonomous Underwater Vehicle (AUV) Amarine — FILKOM Universitas Brawijaya**

---

## 📑 Daftar Isi
1. [Arsitektur Sistem & Aliran Data Digital Twin (HIL & SITL)](#1-️-arsitektur-sistem--aliran-data-digital-twin-hil--sitl)
2. [Sistem Koordinat & Kinematika 6-DOF](#2--sistem-koordinat--kinematika-6-dof)
3. [Perhitungan Lengkap Dinamika Hidrodinamika 6-DOF (Persamaan Fossen)](#3--perhitungan-lengkap-dinamika-hidrodinamika-6-dof-persamaan-fossen)
4. [Perhitungan Matriks Massa Total 6x6 (M = M_RB + M_A) & Inversnya](#4--perhitungan-matriks-massa-total-6x6-m--m_rb--m_a--inversnya)
5. [Perhitungan Matriks Coriolis & Sentripetal 6x6 (C(v))](#5--perhitungan-matriks-coriolis--sentripetal-6x6-cv)
6. [Perhitungan Matriks Redaman Gesekan Air Nonlinier (D_L + D_Q|v|)](#6--perhitungan-matriks-redaman-gesekan-air-nonlinier-d_l--d_qv)
7. [Perhitungan Hidrostatis, Gaya Apung Archimedes & Momen Penegak g(eta)](#7-️-perhitungan-hidrostatis-gaya-apung-archimedes--momen-penegak-geta)
8. [Perhitungan Matriks Alokasi Thruster (TAM 6x6) & Inversi Pseudo-Inverse](#8--perhitungan-matriks-alokasi-thruster-tam-6x6--inversi-pseudo-inverse)
9. [Perhitungan Fusi Sensor Multi-Rate: 15-State Extended Kalman Filter (EKF)](#9-️-perhitungan-fusi-sensor-multi-rate-15-state-extended-kalman-filter-ekf)
10. [Perhitungan Identifikasi Sistem Daring ARMAX & Adaptasi RLS](#10--perhitungan-identifikasi-sistem-daring-armax--adaptasi-rls)
11. [Perhitungan Pengendali Closed-Loop PID + Kompensasi Feedforward](#11--perhitungan-pengendali-closed-loop-pid--kompensasi-feedforward)
12. [Perhitungan Integrasi Numerik Runge-Kutta Orde ke-4 (RK4)](#12-️-perhitungan-integrasi-numerik-runge-kutta-orde-ke-4-rk4)
13. [Model Lingkungan Bawah Air & Gangguan Arus](#13--model-lingkungan-bawah-air--gangguan-arus)
14. [Model Dinamika Thruster T200 BlueRobotics](#14--model-dinamika-thruster-t200-bluerobotics)
15. [Tabel Komprehensif Seluruh Parameter & Satuan SI Terkalibrasi](#15--tabel-komprehensif-seluruh-parameter--satuan-si-terkalibrasi)
16. [Panduan Menjalankan Sistem](#16--panduan-menjalankan-sistem)
17. [Struktur Kode Sumber & Arsitektur Perangkat Lunak](#17--struktur-kode-sumber--arsitektur-perangkat-lunak)

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

### Aliran Data Pipeline per Frame (50 Hz):

```
Sensor Data ──→ EKF Predict ──→ EKF Update ──→ SysID (RLS) ──→ PID Controller
    │                │              │               │                │
    │           IMU 100Hz      Depth 20Hz     Adapts M,D         cmd_vel
    │                │         DVL 10Hz           │                │
    ▼                ▼              ▼              ▼                ▼
ROS2 Topics    State x(15)    Corrected x    Updated θ     TAM Allocation
                                                                   │
                                                                   ▼
                                                          Thruster Forces
                                                                   │
                                                                   ▼
                                                         Fossen Dynamics
                                                         (RK4 Integration)
                                                                   │
                                                                   ▼
                                                        3D Viewport Render
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
  ```
  eta = [x, y, z, phi, theta, psi]^T
  ```
  - x = posisi utara (m), y = posisi timur (m), z = kedalaman (m)
  - phi = sudut roll (rad), theta = sudut pitch (rad), psi = sudut yaw/haluan (rad)

- **Vektor Kecepatan Linier & Kecepatan Sudut dalam Kerangka Bodi {b}**:
  ```
  nu = [u, v, w, p, q, r]^T
  ```
  - u = kecepatan maju (m/s), v = kecepatan geser samping (m/s), w = kecepatan selam (m/s)
  - p = laju putar roll (rad/s), q = laju putar pitch (rad/s), r = laju putar yaw (rad/s)

- **Vektor Gaya & Momen Generalisasi dalam Kerangka Bodi {b}**:
  ```
  tau = [X, Y, Z, K, M, N]^T
  ```
  - X = gaya maju (N), Y = gaya geser samping (N), Z = gaya selam (N)
  - K = momen roll (N·m), M = momen pitch (N·m), N = momen yaw (N·m)

---

### C. Persamaan Transformasi Kinematika Euler:

Turunan posisi bumi `eta_dot` dihitung dari kecepatan bodi `nu`:
```
[x_dot, y_dot, z_dot]^T = R_b_to_n * [u, v, w]^T
[phi_dot, theta_dot, psi_dot]^T = T_Theta * [p, q, r]^T
```

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
```
q_dot = 0.5 * Omega * q
```

Di mana matriks Omega adalah:
```
Omega = [
  [  0, -p, -q, -r ],
  [  p,  0,  r, -q ],
  [  q, -r,  0,  p ],
  [  r,  q, -p,  0 ]
]
```

> **Catatan Implementasi:** Dalam kode (`Kinematics.js`), digunakan integrasi kuaternion dengan normalisasi ulang untuk menjaga `|q| = 1` setiap langkah waktu. Rotasi bodi→dunia dan dunia→bodi diimplementasikan langsung menggunakan operasi kuaternion tanpa konversi ke matriks.

---

## 3. 🌊 Perhitungan Lengkap Dinamika Hidrodinamika 6-DOF (Persamaan Fossen)

Persamaan gerak dinamika maritim nonlinier Fossen (Fossen, 2021, Bab 6):

```
M * nu_dot + C(nu) * nu + D(nu_r) * nu_r + g(eta) = tau_thruster + tau_env + tau_pinn
```

Di mana:
- **M**: Matriks Massa Inersia Total (6x6) = Massa Bodi Kaku (M_RB) + Massa Tambah Fluida (M_A)
- **C(nu)**: Matriks Gaya Coriolis & Sentripetal (6x6) — Persamaan 6.43 Fossen
- **D(nu_r)**: Matriks Redaman Hidrodinamika Fluida (6x6) — Linier + Kuadratik
- **g(eta)**: Vektor Gaya & Momen Pemulih Hidrostatis (6x1) — Persamaan 4.14 Fossen
- **nu_r = nu - nu_current**: Kecepatan relatif terhadap arus air
- **tau_thruster**: Gaya dorong total dari 6 unit motor (via Thruster Allocation Matrix)
- **tau_env**: Gangguan gaya lingkungan (arus, gelombang — model Gauss-Markov)
- **tau_pinn**: Kompensasi residual dari Physics-Informed Neural Network (PINN)

### Solusi Percepatan:
```
nu_dot = M^(-1) * [tau_thruster + tau_pinn - C(nu)*nu - D(nu_r)*nu_r - g(eta)]
```

> **Referensi Implementasi:** File `src/dt-core/HydrodynamicsEngine.js`, metode `computeAccelerations()` pada baris 320.

---

## 4. 🧮 Perhitungan Matriks Massa Total 6x6 (M = M_RB + M_A) & Inversnya

### A. Matriks Massa Bodi Kaku (M_RB):
Diketahui parameter robot (dari `VehicleConfig.js`):
- Massa kering kapal: `m = 11.5 kg`
- Pusat gravitasi: `CG = [xG, yG, zG] = [0.0, 0.0, 0.0] m` (asal kerangka bodi)
- Momen Inersia Roll: `Ixx = 0.12 kg·m²`
- Momen Inersia Pitch: `Iyy = 0.22 kg·m²`
- Momen Inersia Yaw: `Izz = 0.24 kg·m²`
- Produk Inersia: `Ixy = Ixz = Iyz = 0.0` (simetris bilateral)

Formulasi Fossen (2021, Pers. 3.42) — karena CG berada di asal {b}, maka elemen kopling menjadi nol:

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
Air di sekitar lambung yang ikut terdorong memiliki inersia terkalibrasi (Fossen 2021, Pers. 6.38):
- Added mass Surge: `Xu_dot = 5.5 kg` — arah memanjang, profil aerodinamis
- Added mass Sway: `Yv_dot = 12.7 kg` — badan samping lebar, resistansi lateral tinggi
- Added mass Heave: `Zw_dot = 14.6 kg` — pelat atas/bawah datar, profil resistansi vertikal besar
- Added inertia Roll: `Kp_dot = 0.12 kg·m²`
- Added inertia Pitch: `Mq_dot = 0.12 kg·m²`
- Added inertia Yaw: `Nr_dot = 0.12 kg·m²`

Sumber: Studi empiris BlueROV2 — Berg (2012), Wu (2018), Fossen (2021).

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

Perhitungan elemen per elemen:
- `M[0][0] = 11.5 + 5.5  = 17.0 kg` (efektif massa surge)
- `M[1][1] = 11.5 + 12.7 = 24.2 kg` (efektif massa sway)
- `M[2][2] = 11.5 + 14.6 = 26.1 kg` (efektif massa heave)
- `M[3][3] = 0.12 + 0.12 = 0.24 kg·m²` (efektif inersia roll)
- `M[4][4] = 0.22 + 0.12 = 0.34 kg·m²` (efektif inersia pitch)
- `M[5][5] = 0.24 + 0.12 = 0.36 kg·m²` (efektif inersia yaw)

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

**Interpretasi Fisik:** Kapal terasa 1.48× lebih berat saat bergerak maju (surge), tetapi 2.27× lebih berat saat bergerak menyelam (heave) karena added mass vertikal sangat besar akibat profil datar badan wahana.

---

### D. Invers Matriks Massa Total (M^-1):
Karena M diagonal, inversnya adalah reciprocal tiap elemen diagonal:

```
Perhitungan invers:
  M_inv[0][0] = 1 / 17.0  = 0.05882 (1/kg → percepatan surge per Newton)
  M_inv[1][1] = 1 / 24.2  = 0.04132 (1/kg → percepatan sway per Newton)
  M_inv[2][2] = 1 / 26.1  = 0.03831 (1/kg → percepatan heave per Newton)
  M_inv[3][3] = 1 / 0.24  = 4.16667 (1/(kg·m²) → percepatan putar roll per N·m)
  M_inv[4][4] = 1 / 0.34  = 2.94118 (1/(kg·m²) → percepatan putar pitch per N·m)
  M_inv[5][5] = 1 / 0.36  = 2.77778 (1/(kg·m²) → percepatan putar yaw per N·m)
```

```
M^-1 = [
  [ 0.05882,  0.00000,  0.00000,  0.00000,  0.00000,  0.00000 ],
  [ 0.00000,  0.04132,  0.00000,  0.00000,  0.00000,  0.00000 ],
  [ 0.00000,  0.00000,  0.03831,  0.00000,  0.00000,  0.00000 ],
  [ 0.00000,  0.00000,  0.00000,  4.16667,  0.00000,  0.00000 ],
  [ 0.00000,  0.00000,  0.00000,  0.00000,  2.94118,  0.00000 ],
  [ 0.00000,  0.00000,  0.00000,  0.00000,  0.00000,  2.77778 ]
]
```

**Interpretasi:** Gaya 1 Newton pada arah surge menghasilkan percepatan 0.059 m/s², sedangkan momen 1 N·m pada sumbu roll menghasilkan percepatan sudut 4.167 rad/s² — wahana sangat sensitif terhadap momen putar.

> **Referensi Implementasi:** File `src/dt-core/HydrodynamicsEngine.js`, metode `computeMassMatrix()` baris 145-180, dan `invert6x6()` baris 96-140 (Gauss-Jordan elimination dengan partial pivoting).

---

## 5. 🔄 Perhitungan Matriks Coriolis & Sentripetal 6x6 (C(v))

Matriks Coriolis dimodelkan menggunakan matriks perkalian silang *skew-symmetric* `S(a)` (Fossen 2021, Bab 3.3):
```
S(a) = [
  [   0,  -a3,   a2 ],
  [  a3,    0,  -a1 ],
  [ -a2,   a1,    0 ]
]

Sifat: S(a) * b = a × b (cross product)
```

### Struktur Matriks Coriolis:
```
C(nu) = C_RB(nu) + C_A(nu_r)
```

Di mana vektor momentumnya adalah:
- **Momentum linier bodi kaku:**
  ```
  a_RB = m * [u, v, w] = [11.5*u, 11.5*v, 11.5*w]
  ```
- **Momentum sudut bodi kaku:**
  ```
  b_RB = [Ixx*p, Iyy*q, Izz*r] = [0.12*p, 0.22*q, 0.24*r]
  ```
- **Momentum linier added mass (relatif terhadap arus):**
  ```
  a_A = [Xu_dot*u_r, Yv_dot*v_r, Zw_dot*w_r] = [5.5*u_r, 12.7*v_r, 14.6*w_r]
  ```
- **Momentum sudut added mass:**
  ```
  b_A = [Kp_dot*p, Mq_dot*q, Nr_dot*r] = [0.12*p, 0.12*q, 0.12*r]
  ```

### Implementasi Coriolis (Fossen 2021, Sec 6.5):

Untuk wahana bertipe *open-frame bluff-body ROV* (seperti BlueROV2), kopling potensial antara kecepatan linier surge/sway dan laju putar sudut bersifat negligible dibandingkan dengan gaya hambatan viskus. Menyertakan blok kopling penuh akan menimbulkan *Munk moment* buatan yang tidak stabil tanpa adanya lifting surface.

Oleh karena itu, implementasi hanya menyertakan blok rotasional gyroscopic:

```
C(nu) = [
  [  0_{3x3},                0_{3x3}           ],
  [  0_{3x3},              -S(b_RB + b_A)      ]
]
```

### Contoh Hitungan Numerik:
Jika kapal berputar: `p = 0.1 rad/s`, `q = 0.05 rad/s`, `r = 0.3 rad/s`:

```
b_RB = [0.12×0.1, 0.22×0.05, 0.24×0.3] = [0.012, 0.011, 0.072]
b_A  = [0.12×0.1, 0.12×0.05, 0.12×0.3] = [0.012, 0.006, 0.036]
b_total = [0.024, 0.017, 0.108]

S(b_total) = [
  [  0.000, -0.108,  0.017 ],
  [  0.108,  0.000, -0.024 ],
  [ -0.017,  0.024,  0.000 ]
]

Blok C[3..5][3..5] = -S(b_total) = [
  [  0.000,  0.108, -0.017 ],
  [ -0.108,  0.000,  0.024 ],
  [  0.017, -0.024,  0.000 ]
]
```

> **Referensi Implementasi:** File `src/dt-core/HydrodynamicsEngine.js`, metode `computeCoriolisMatrix()` baris 189-246.

---

## 6. 🌊 Perhitungan Matriks Redaman Gesekan Air Nonlinier (D_L + D_Q|v|)

Gaya dan momen hambatan air dihitung dari kombinasi redaman linier (*skin friction*) dan redaman kuadratik (*vortex shedding drag*). Koefisien diperoleh dari uji pool dan data empiris (Berg 2012, Wu 2018):

### Persamaan Tiap Sumbu:

| No. | Sumbu | Persamaan Gaya/Momen Hambatan | Koef. Linier | Koef. Kuadratik |
|:---:|:------|:-----------------------------|:-------------|:----------------|
| 1 | **Surge (Maju)** | `F = -(4.03 × u + 18.18 × |u| × u)` | Xu = 4.03 N·s/m | Xuu = 18.18 N·s²/m² |
| 2 | **Sway (Samping)** | `F = -(6.22 × v + 21.66 × |v| × v)` | Yv = 6.22 N·s/m | Yvv = 21.66 N·s²/m² |
| 3 | **Heave (Selam)** | `F = -(5.18 × w + 36.99 × |w| × w)` | Zw = 5.18 N·s/m | Zww = 36.99 N·s²/m² |
| 4 | **Roll (Guling)** | `M = -(0.07 × p + 1.55 × |p| × p)` | Kp = 0.07 N·m·s/rad | Kpp = 1.55 N·m·s²/rad² |
| 5 | **Pitch (Angguk)** | `M = -(0.07 × q + 1.55 × |q| × q)` | Mq = 0.07 N·m·s/rad | Mqq = 1.55 N·m·s²/rad² |
| 6 | **Yaw (Belok)** | `M = -(0.07 × r + 1.55 × |r| × r)` | Nr = 0.07 N·m·s/rad | Nrr = 1.55 N·m·s²/rad² |

### Contoh Hitungan Nyata — Kasus 1 (Maju + Belok):
Jika kapal bergerak maju `u = 0.6 m/s` dan belok `r = 0.3 rad/s`:

```
F_drag_surge = -(4.03 × 0.6 + 18.18 × |0.6| × 0.6)
             = -(4.03 × 0.6 + 18.18 × 0.6 × 0.6)
             = -(2.418 + 18.18 × 0.36)
             = -(2.418 + 6.545)
             = -8.963 Newton

M_drag_yaw   = -(0.07 × 0.3 + 1.55 × |0.3| × 0.3)
             = -(0.07 × 0.3 + 1.55 × 0.3 × 0.3)
             = -(0.021 + 1.55 × 0.09)
             = -(0.021 + 0.1395)
             = -0.1605 N·m
```

### Contoh Hitungan Nyata — Kasus 2 (Menyelam Vertikal):
Jika kapal menyelam `w = 0.4 m/s`:

```
F_drag_heave = -(5.18 × 0.4 + 36.99 × |0.4| × 0.4)
             = -(5.18 × 0.4 + 36.99 × 0.4 × 0.4)
             = -(2.072 + 36.99 × 0.16)
             = -(2.072 + 5.918)
             = -7.990 Newton
```

### Kecepatan Terminal (Terminal Velocity):

Pada kecepatan terminal, gaya hambatan total sama dengan gaya pendorong. Untuk masing-masing sumbu:

| Sumbu | Gaya Maks Thruster | Kecepatan Terminal |
|:------|:-------------------|:-------------------|
| **Surge** | 50.0 N × 4 × cos(45°) = 141.4 N total | ~1.35 m/s |
| **Heave** | 50.0 N × 2 = 100.0 N total | ~0.80 m/s |

> **Referensi Implementasi:** File `src/dt-core/HydrodynamicsEngine.js`, metode `computeDampingForces()` baris 255-270. Koefisien dari `VehicleConfig.js` baris 129-149.

---

## 7. ⚖️ Perhitungan Hidrostatis, Gaya Apung Archimedes & Momen Penegak g(eta)

### A. Parameter Fisik:

| Parameter | Simbol | Nilai | Satuan | Sumber |
|:----------|:-------|:------|:-------|:-------|
| Massa kering kapal | m | 11.5 | kg | Penimbangan digital darat |
| Percepatan gravitasi | g | 9.80665 | m/s² | Konstanta standar |
| Volume benaman air | V | 0.01151 | m³ (11.51 L) | Uji benaman dengan ballast netral |
| Densitas air kolam (26°C) | ρ | 996.78 | kg/m³ | UNESCO 1980 (T=26°C, S=0 PSU) |
| Pusat gravitasi (CG) | [xG, yG, zG] | [0, 0, 0] | m | Asal kerangka bodi {b} |
| Pusat apung (CB) | [xB, yB, zB] | [0, 0, -0.025] | m | 25mm di atas CG (NED) |

### B. Perhitungan Keseimbangan Vertikal Archimedes:

**1. Berat Total Kapal (W):**
```
W = m × g
  = 11.5 kg × 9.80665 m/s²
  = 112.7765 Newton
```

**2. Gaya Apung Fluida (B):**
```
B = ρ × g × V
  = 996.78 kg/m³ × 9.80665 m/s² × 0.01151 m³
  = 996.78 × 9.80665 × 0.01151
  = 112.4908 Newton
```

> **Catatan Densitas Air:** Pada suhu kolam 26°C, densitas air tawar dihitung menggunakan rumus Kell (1975) yang diimplementasikan dalam `EnvironmentModel.js`:
> ```
> ρ(26°C) = 999.842594 + 6.794×10⁻² × 26 - 9.095×10⁻³ × 26² + ... = 996.78 kg/m³
> ```
> Ini lebih rendah daripada ρ = 998.2 kg/m³ pada 20°C.

**3. Gaya Bersih ke Atas (Delta F):**
```
ΔF = B - W
   = 112.4908 - 112.7765
   = -0.2857 Newton
```

**Interpretasi:** Gaya bersih negatif (wahana sedikit lebih berat daripada gaya apung) berarti wahana cenderung tenggelam perlahan tanpa thrust — **near-neutral buoyancy** yang ideal untuk operasi bawah air (berat 29 gram-force lebih dari apungannya).

> **Catatan Kalibrasi:** Volume benaman 0.01151 m³ sudah dikalibrasi dengan ballast weights untuk mencapai kondisi mendekati netral buoyancy. Dalam suhu kolam yang berbeda, densitas air berubah dan sedikit menggeser keseimbangan ini.

---

### C. Vektor Gaya & Momen Pemulih Hidrostatis g(eta):

Formulasi Fossen (2021, Pers. 4.14):
```
g(eta) = [
  (W - B) × sin(theta),
  -(W - B) × cos(theta) × sin(phi),
  -(W - B) × cos(theta) × cos(phi),
  -(yG×W - yB×B) × cos(theta) × cos(phi) + (zG×W - zB×B) × cos(theta) × sin(phi),
  (zG×W - zB×B) × sin(theta) + (xG×W - xB×B) × cos(theta) × cos(phi),
  -(xG×W - xB×B) × cos(theta) × sin(phi) - (yG×W - yB×B) × sin(theta)
]
```

### D. Substitusi Angka Robot:

Karena `xG=xB=0`, `yG=yB=0`, `zG=0`, `zB=-0.025`:

```
Hitung (W - B):
  W - B = 112.7765 - 112.4908 = +0.2857 N

Hitung (zG×W - zB×B):
  = (0.0 × 112.7765) - (-0.025 × 112.4908)
  = 0 + 2.8123
  = +2.8123 N·m

Hitung (yG×W - yB×B) = (0×W - 0×B) = 0
Hitung (xG×W - xB×B) = (0×W - 0×B) = 0
```

Maka vektor restoring simplifikasi:
```
g(eta) = [
  +0.2857 × sin(theta),              ← gaya surge (sangat kecil karena near-neutral)
  -0.2857 × cos(theta) × sin(phi),   ← gaya sway
  -0.2857 × cos(theta) × cos(phi),   ← gaya heave
  +2.8123 × cos(theta) × sin(phi),   ← momen penegak roll
  +2.8123 × sin(theta),              ← momen penegak pitch
  0.0                                ← tidak ada momen yaw dari restoring
]
```

### E. Momen Penegak (Righting Moment):

- **Momen Penegak Roll:** `K_g = +2.8123 × sin(phi) N·m`
  - Jika kapal miring 10° (phi = 0.1745 rad): `K_g = 2.8123 × 0.1736 = 0.488 N·m` → mengembalikan ke posisi datar
- **Momen Penegak Pitch:** `M_g = +2.8123 × sin(theta) N·m`
  - Jika hidung menukik 15° (theta = 0.2618 rad): `M_g = 2.8123 × 0.2588 = 0.728 N·m` → mengembalikan mendatar

**Interpretasi Fisik:** Momen penegak timbul karena pusat apung (CB) berada 25mm di atas pusat gravitasi (CG). Pasangan gaya W dan B menciptakan *righting moment* yang secara pasif menstabilkan wahana — wahana secara alami cenderung kembali ke orientasi mendatar tanpa bantuan thruster.

> **Referensi Implementasi:** File `src/dt-core/HydrodynamicsEngine.js`, metode `computeRestoringForces()` baris 285-310.

---

## 8. 🌀 Perhitungan Matriks Alokasi Thruster (TAM 6x6) & Inversi Pseudo-Inverse

### A. Geometri Posisi & Vektor Arah 6 Motor Thruster:

| Motor | Posisi [x, y, z] (m) | Vektor Gaya [dx, dy, dz] | Fungsi Gerakan |
| :--- | :--- | :--- | :--- |
| **T1 (Depan-Kiri)** | [+0.14, +0.11, 0.0] | [cos(45°), -sin(45°), 0] = [+0.7071, -0.7071, 0] | Maju (+), Geser Kiri (-), Yaw Kanan (+) |
| **T2 (Depan-Kanan)**| [+0.14, -0.11, 0.0] | [cos(45°), +sin(45°), 0] = [+0.7071, +0.7071, 0] | Maju (+), Geser Kanan (+), Yaw Kiri (-) |
| **T3 (Belakang-Kiri)**| [-0.14, +0.11, 0.0] | [cos(45°), +sin(45°), 0] = [+0.7071, +0.7071, 0] | Maju (+), Geser Kanan (+), Yaw Kanan (+) |
| **T4 (Belakang-Kanan)**| [-0.14, -0.11, 0.0] | [cos(45°), -sin(45°), 0] = [+0.7071, -0.7071, 0] | Maju (+), Geser Kiri (-), Yaw Kiri (-) |
| **T5 (Vertikal Depan)**| [+0.18, 0.0, 0.0] | [0, 0, -1.0] | Selam (thrust ke bawah → gaya ke atas di NED) |
| **T6 (Vertikal Belakang)**| [-0.18, 0.0, 0.0] | [0, 0, -1.0] | Selam (thrust ke bawah → gaya ke atas di NED) |

### B. Perhitungan Momen Yaw (Lengan Momen):

Untuk thruster horizontal, momen yaw dihitung dari cross product `r × F`:
```
N_j = x_j × dy_j - y_j × dx_j

T1: N = (+0.14)(−0.7071) − (+0.11)(+0.7071) = −0.0990 − 0.0778 = −0.1768
    Tapi karena gaya T1 menghasilkan yaw CW: N = +0.14×(−0.7071) − (+0.11)×(+0.7071)
    = 0.14×(−0.7071) − 0.11×0.7071
    Sebenarnya:
    N_j = px × dy - py × dx
    T1: N = 0.14 × (-0.7071) - 0.11 × 0.7071 = -0.0990 - 0.0778 = -0.1768

    ❌ Revisi: menggunakan rumus cross product dari kode (VehicleConfig.js, baris 362):
    B[5][j] = px × dy - py × dx

    T1: 0.14 × (-0.7071) - 0.11 × (0.7071) = -0.0990 - 0.0778 = -0.1768  
    T2: 0.14 × (0.7071)  - (-0.11) × (0.7071) = 0.0990 + 0.0778 = +0.1768
    T3: (-0.14) × (0.7071) - 0.11 × (0.7071)  = -0.0990 - 0.0778 = -0.1768  
    T4: (-0.14) × (-0.7071) - (-0.11) × (0.7071) = 0.0990 + 0.0778 = +0.1768
```

Wait — mari kita hitung ulang dengan benar menggunakan kode sumber:
```
B[5][j] = pos[j].x × dir[j].y - pos[j].y × dir[j].x

T1: (+0.14)×(-0.7071) - (+0.11)×(+0.7071) = -0.0990 - 0.0778 = -0.1768
T2: (+0.14)×(+0.7071) - (-0.11)×(+0.7071) = +0.0990 + 0.0778 = +0.1768
T3: (-0.14)×(+0.7071) - (+0.11)×(+0.7071) = -0.0990 - 0.0778 = -0.1768
T4: (-0.14)×(-0.7071) - (-0.11)×(+0.7071) = +0.0990 + 0.0778 = +0.1768
T5: (vertikal, arah [0,0,-1]) → N = (+0.18)×0 - 0×0 = 0
T6: (vertikal, arah [0,0,-1]) → N = (-0.18)×0 - 0×0 = 0
```

### C. Perhitungan Momen Pitch (Lengan Momen Vertikal):
```
B[4][j] = pos[j].z × dir[j].x - pos[j].x × dir[j].z

T5: 0×0 - (+0.18)×(-1) = +0.18
T6: 0×0 - (-0.18)×(-1) = -0.18
```

### D. Matriks Konfigurasi Alokasi Thruster T_alloc (6x6):
`tau = T_alloc × u_T`

```
           T1       T2       T3       T4       T5       T6
T_alloc = [
  [ +0.7071, +0.7071, +0.7071, +0.7071,  0.0000,  0.0000 ],  ← X (surge)
  [ -0.7071, +0.7071, +0.7071, -0.7071,  0.0000,  0.0000 ],  ← Y (sway)
  [  0.0000,  0.0000,  0.0000,  0.0000, -1.0000, -1.0000 ],  ← Z (heave)
  [  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000 ],  ← K (roll)
  [  0.0000,  0.0000,  0.0000,  0.0000, +0.1800, -0.1800 ],  ← M (pitch)
  [ -0.1768, +0.1768, -0.1768, +0.1768,  0.0000,  0.0000 ]   ← N (yaw)
]
```

> **Catatan Baris Roll (K):** Momen roll = 0 untuk semua thruster karena semua thruster terletak pada bidang z=0. Stabilisasi roll dilakukan secara pasif oleh momen penegak hidrostatis (CB di atas CG).

---

### E. Solusi Closed-Form Inversi Alokasi Tenaga Motor:
`u_T = T_alloc_pseudo_inverse × tau_command`

Perhitungan daya tiap motor dari perintah gaya `[Fx, Fy, Fz, K, My, Mz]`:

```
Horizontal thrusters (dari 4×4 sub-matriks surge/sway/yaw):
  T1 = (0.3536 × Fx) - (0.3536 × Fy) - (1.4141 × Mz)
  T2 = (0.3536 × Fx) + (0.3536 × Fy) + (1.4141 × Mz)
  T3 = (0.3536 × Fx) + (0.3536 × Fy) - (1.4141 × Mz)
  T4 = (0.3536 × Fx) - (0.3536 × Fy) + (1.4141 × Mz)

Vertical thrusters (dari 2×2 sub-matriks heave/pitch):
  T5 = (-0.5000 × Fz) + (2.7778 × My)
  T6 = (-0.5000 × Fz) - (2.7778 × My)
```

### F. Contoh Hitungan Alokasi Thruster:

**Perintah: Maju penuh (Fx = 100 N) sambil belok kanan (Mz = 5 N·m):**
```
T1 = 0.3536 × 100 - 1.4141 × 5 = 35.36 - 7.07 = 28.29 N
T2 = 0.3536 × 100 + 1.4141 × 5 = 35.36 + 7.07 = 42.43 N
T3 = 0.3536 × 100 - 1.4141 × 5 = 35.36 - 7.07 = 28.29 N
T4 = 0.3536 × 100 + 1.4141 × 5 = 35.36 + 7.07 = 42.43 N
T5 = 0 N (tidak ada perintah heave/pitch)
T6 = 0 N

Total thrust: 141.4 N maju, dengan pasangan kanan (T2,T4) lebih kuat dari kiri (T1,T3) → kapal belok kanan.
```

> **Referensi Implementasi:** File `src/dt-core/VehicleConfig.js`, metode `computeAllocationMatrix()` baris 345-366.

---

## 9. 🛰️ Perhitungan Fusi Sensor Multi-Rate: 15-State Extended Kalman Filter (EKF)

### A. Vektor Keadaan EKF (x berukuran 15x1):

```
x = [
  x[0..2]:   Posisi NED           [x, y, z]          (meter)
  x[3..5]:   Kecepatan bodi       [u, v, w]          (meter/detik)
  x[6..8]:   Sudut Euler          [phi, theta, psi]  (radian)
  x[9..11]:  Bias drift akselerometer [b_ax, b_ay, b_az]  (m/s²)
  x[12..14]: Bias drift giroskop  [b_gx, b_gy, b_gz] (rad/s)
]
```

**Mengapa 15 state?** Bias sensor MEMS (akselerometer dan giroskop) bergeser (*drift*) secara perlahan seiring waktu dan suhu. Dengan mengestimasi bias secara online, EKF dapat mengompensasi drift secara otomatis sehingga estimasi posisi tetap akurat.

---

### B. Tahap Prediksi (Time-Update Step — Frekuensi 100 Hz):
Interval waktu IMU: `dt = 0.01 detik`.
Input sensor: akselerasi spesifik terukur `f_m = [ax, ay, az]` dan laju putar terukur `omega_m = [gx, gy, gz]`.

**1. Prediksi Keadaan:**
```
posisi_dot   = R_b_to_n(Theta) × v_body
kecepatan_dot = (f_m - b_a) - S(omega_m - b_g) × v_body + R_n_to_b × [0, 0, g]
sudut_dot    = T_Theta(Theta) × (omega_m - b_g)
bias_a_dot   = 0  (model random walk drift)
bias_g_dot   = 0  (model random walk drift)
```

**2. Propagasi Kovarians Ketidakpastian (15x15):**
```
P_k = Phi_k × P_{k-1} × Phi_k^T + Q_k
```
Di mana `Phi_k = I_15 + F_k × dt` dengan Jacobian sistem `F_k = d(f) / d(x)`.

### Matriks Noise Proses Q_k (15x15 diagonal):
```
Q = diag([
  σ_pos²×dt,  σ_pos²×dt,  σ_pos²×dt,     ← ketidakpastian posisi
  σ_vel²×dt,  σ_vel²×dt,  σ_vel²×dt,     ← ketidakpastian kecepatan
  σ_att²×dt,  σ_att²×dt,  σ_att²×dt,     ← ketidakpastian sudut
  σ_ba²×dt,   σ_ba²×dt,   σ_ba²×dt,      ← drift bias akselerometer
  σ_bg²×dt,   σ_bg²×dt,   σ_bg²×dt       ← drift bias giroskop
])
```

---

### C. Tahap Koreksi Pengukuran (Measurement-Update Step):
Saat sensor kedalaman (20 Hz) atau DVL (10 Hz) masuk:

**1. Residual Inovasi:**
```
y_tilde = z_meas - h(x_pred)
```

**2. Kovarians Inovasi:**
```
S = H × P × H^T + R
```

**3. Validasi Outlier Chi-Square Gating (Mahalanobis Distance):**
```
d_M² = y_tilde^T × S^-1 × y_tilde

Jika d_M² > gamma_threshold → tolak pengukuran (outlier)
Threshold γ = 9.0 untuk sensor 1D (chi-squared df=1, p=0.003)
```

**Interpretasi:** Sensor yang memberikan data jauh dari prediksi (misalnya spike noise atau gangguan fisik) akan dideteksi dan ditolak secara otomatis oleh gating ini.

**4. Kalman Gain Optimal:**
```
K = P × H^T × S^-1
```

**5. Update State & Kovarians (Bentuk Joseph — numerically stable):**
```
x_updated = x_pred + K × y_tilde

P_updated = (I - K×H) × P × (I - K×H)^T + K × R × K^T
```

> **Mengapa Joseph Form?** Dibandingkan formula standar `P = (I - KH)P`, Joseph Form menjamin matriks P tetap simetris positif-definit meskipun ada error pembulatan floating-point. Ini krusial untuk stabilitas numerik EKF jangka panjang.

> **Referensi Implementasi:** File `src/dt-core/StateEstimator.js`.

---

## 10. 🧠 Perhitungan Identifikasi Sistem Daring ARMAX & Adaptasi RLS

### A. Model Polinomial Diskrit Input-Output:

Model ARMAX (*AutoRegressive Moving Average with eXogenous input*):
```
A(q⁻¹) × y(t) = B(q⁻¹) × u(t-d) + C(q⁻¹) × e(t)
```

Di mana:
- `y(t)` = output terukur (kecepatan wahana, m/s)
- `u(t)` = input perintah thruster (normalized -1 to +1)
- `e(t)` = gangguan white noise
- `d = 1` = delay transport (1 langkah waktu)

Bentuk regresi linier:
```
y(t) = phi(t)^T × theta + e(t)
```

- **Vektor Regresor (4×1):**
  ```
  phi(t) = [ -y(t-1),  -y(t-2),  u(t-1),  u(t-2) ]^T
  ```
- **Vektor Parameter yang Diestimasi (4×1):**
  ```
  theta(t) = [ a1, a2, b0, b1 ]^T
  ```

### B. Parameter Awal dari Data Bollard Test T200 @ 16V:

Sampling rate: `Ts = 0.05 s` (20 Hz)
Time constant motor fisik: `tau_m = 0.42 s`
Pole diskrit: `z = e^(-Ts/tau_m) = e^(-0.05/0.42) = 0.888`

| Model | Parameter | Nilai Awal | Fit Rate |
|:------|:----------|:-----------|:---------|
| **ARX** | a1=-0.82, a2=0.06, b0=0.14, b1=0.08 | Dari bollard test | 89.2% |
| **ARMAX** | a1=-0.80, a2=0.05, b0=0.15, b1=0.08, c1=0.08 | Dari bollard test | 94.5% |
| **OE** | f1=-0.78, f2=0.04, b0=0.16, b1=0.08 | Dari bollard test | 92.4% |
| **Box-Jenkins** | f1=-0.78, f2=0.04, b0=0.16, b1=0.08, c1=0.06, d1=-0.40 | Dari bollard test | 96.8% |

### C. Persamaan Komputasi RLS dengan Forgetting Factor:

Forgetting factor: `lambda = 0.985` (adapts over ~67 samples = 3.35 detik @ 20Hz)

**1. Gain Adaptif K(t):**
```
K(t) = P(t-1) × phi(t) / (lambda + phi(t)^T × P(t-1) × phi(t))
```

**2. Error Prediksi a Priori:**
```
epsilon(t) = y(t) - phi(t)^T × theta(t-1)
```

**3. Pembaruan Parameter:**
```
theta(t) = theta(t-1) + K(t) × epsilon(t)
```

**4. Pembaruan Matriks Kovarians P(t) (4×4):**
```
P(t) = (1 / lambda) × (I_4 - K(t) × phi(t)^T) × P(t-1)
```

### D. Constraint Stabilitas (Pole Bounding):

Setelah setiap update RLS, parameter dibatasi agar sistem tetap stabil:
```
a1 ∈ [-0.98, 0.00]     ← pole kausal negatif untuk respons stabil
a2 ∈ [0.00, 0.20]      ← damping orde-2 kecil positif
b0 ∈ [0.02, 0.40]      ← gain fisik harus positif
b1 ∈ [0.01, 0.30]      ← gain fisik harus positif
```

### E. Contoh Hitungan Numerik RLS:

**State awal:**
```
theta = [-0.80, 0.05, 0.15, 0.08]
P = 1000 × I_4 (high initial uncertainty)
lambda = 0.985
```

**Pengukuran masuk:** `y_real = 0.65 m/s`, histori: `y(t-1) = 0.5`, `y(t-2) = 0.3`, `u(t-1) = 0.8`, `u(t-2) = 0.6`

```
phi = [-0.5, -0.3, 0.8, 0.6]

Prediksi: phi^T × theta = (-0.5)×(-0.80) + (-0.3)×0.05 + 0.8×0.15 + 0.6×0.08
                         = 0.40 - 0.015 + 0.12 + 0.048
                         = 0.553

y_norm = 0.65 / 1.3 = 0.50 (normalized)

Error: epsilon = 0.50 - 0.553 = -0.053

→ K(t) dihitung → theta di-update → P di-update → model bertambah akurat
```

> **Referensi Implementasi:** File `src/services/SystemIdentificationEngine.js`, metode `rlsUpdate()` baris 215-324.

---

## 11. 🎯 Perhitungan Pengendali Closed-Loop PID + Kompensasi Feedforward

### A. Persamaan Umum PID:
```
Output = Feedforward + Kp × Error + Ki × ∫(Error)dt + Kd × d(Error)/dt
```

### B. Pengendali Kedalaman (Heave Axis):

Parameter PID kedalaman (dari `AUVMotionController.js`):
```
Kp = 0.5
Ki = 0.1
Kd = 0.2
Setpoint awal: 0.8 m
Range output: [-1.0, +1.0] (normalized)
```

Batas kedalaman operasi: `0.15 m – 1.85 m`

**Contoh:** Kapal berada di kedalaman 0.6 m, target 0.8 m:
```
Error = 0.8 - 0.6 = +0.2 m (terlalu dangkal, harus menyelam)
Output = 0.5 × 0.2 + 0.1 × ∫(0.2)dt + 0.2 × d(0.2)/dt
       = 0.10 + integral_term + derivative_term
       ≈ 0.10 (proportional dominan saat pertama kali)

Skala ke thrust vertikal: targetHeave = output × 0.45 = 0.045 m/s
Perintah thruster vertikal: tHeave = (0.045 / 0.45) × 75 = 7.5% power
```

### C. Pengendali Heading (Yaw Axis):

```
Kp = 0.35
Ki = 0.03
Kd = 0.15
Range output: [-1.0, +1.0]
```

**Fitur Course Lock:** Saat user tidak menekan tombol yaw, heading target dikunci ke heading terakhir dan PID heading aktif. Ini memastikan wahana berjalan lurus (*laser-straight*) tanpa drift yaw.

### D. Pengendali Attitude (Roll & Pitch):

```
Kp = 0.8, Ki = 0.0, Kd = 0.3
Range output: [-0.8, +0.8]
Setpoint: 0.0 rad (selalu target datar/level)
```

### E. Strategi Alokasi Thruster (Straight-Line Priority):

```
Prioritas utama: Semua 4 thruster horizontal mendapat daya yang SAMA untuk thrust lurus.
Steering: Diterapkan sebagai differential kecil (maks ±12% saat kecepatan tinggi, ±28% saat lambat).

basePower = surgeNorm × 78%              ← semua thruster 78% saat full forward
yawDiff   = yawNorm × maxDifferential    ← maks 12% differential saat high speed

T1 = basePower + yawDiff - swayPower     ← Front-Left
T2 = basePower - yawDiff + swayPower     ← Front-Right
T3 = basePower + yawDiff + swayPower     ← Rear-Left
T4 = basePower - yawDiff - swayPower     ← Rear-Right

Desaturation: Jika |T_max| > 100%, semua diskalakan secara proporsional.
```

> **Referensi Implementasi:** File `src/services/AUVMotionController.js`, baris 150-240.

---

## 12. ⚙️ Perhitungan Integrasi Numerik Runge-Kutta Orde ke-4 (RK4)

Digital Twin menghitung percepatan wahana pada setiap langkah waktu:

### A. Fungsi Percepatan:
```
f(eta, nu, tau) = M^(-1) × [tau_thruster + tau_pinn - C(nu)×nu - D(nu_r)×nu_r - g(eta)]
```

### B. 4 Tahapan Hitungan RK4 (dt = 0.02 s, 50 Hz):

```
k1 = f(t, eta, nu)
k2 = f(t + 0.5×dt, eta + 0.5×dt×eta_dot_1, nu + 0.5×dt×k1)
k3 = f(t + 0.5×dt, eta + 0.5×dt×eta_dot_2, nu + 0.5×dt×k2)
k4 = f(t + dt, eta + dt×eta_dot_3, nu + dt×k3)

nu_next = nu + (dt / 6) × (k1 + 2×k2 + 2×k3 + k4)
```

### C. Integrasi Posisi & Orientasi:

Posisi diintegrasikan dengan rata-rata tertimbang RK4:
```
pos_dot_avg = (k1_pos + 2×k2_pos + 2×k3_pos + k4_pos) / 6

eta_next[0..2] = eta[0..2] + pos_dot_avg × dt   ← posisi x, y, z
```

Orientasi diintegrasikan menggunakan kuaternion (bukan Euler, menghindari gimbal lock):
```
angRate_avg = (omega_1 + 2×omega_2 + 2×omega_3 + omega_4) / 6

quat_next = integrateQuaternion(quat, angRate_avg, dt)
euler_next = quaternionToEuler(quat_next)

eta_next[3..5] = [euler_next.roll, euler_next.pitch, euler_next.yaw]
```

### D. Mengapa RK4 dan Bukan Euler?

| Metode | Orde | Error per Step | Kestabilan | Beban Komputasi |
|:-------|:-----|:---------------|:-----------|:----------------|
| Euler Eksplisit | 1 | O(dt²) | Tidak stabil untuk dt besar | 1× evaluasi f() |
| Semi-Implicit Euler | 1 | O(dt²) | Sedikit lebih stabil | 1× evaluasi f() |
| **RK4** | **4** | **O(dt⁵)** | **Stabil untuk dt ≤ 0.02s** | **4× evaluasi f()** |

Dengan dt = 0.02s, error RK4 per step ~10⁻¹⁰ vs Euler ~10⁻⁴. Ini krusial untuk simulasi akurat selama misi 10+ menit.

> **Referensi Implementasi:** File `src/dt-core/HydrodynamicsEngine.js`, metode `step()` baris 400-533 (dual integrator: `rk4` dan `semi_implicit`).

---

## 13. 🌡️ Model Lingkungan Bawah Air & Gangguan Arus

### A. Perhitungan Densitas Air (UNESCO 1980 / Kell 1975):

```
ρ_pure(T) = 999.842594 + 6.794×10⁻²×T - 9.095×10⁻³×T² + 1.002×10⁻⁴×T³ - 1.120×10⁻⁶×T⁴ + 6.536×10⁻⁹×T⁵

Koreksi salinitas (S dalam PSU):
ρ(T,S) = ρ_pure + A×S + B×S^(3/2) + C×S²

Di mana:
  A = 0.824493 - 4.090×10⁻³×T + 7.644×10⁻⁵×T² - 8.247×10⁻⁷×T³
  B = -5.725×10⁻³ + 1.023×10⁻⁴×T - 1.655×10⁻⁶×T²
  C = 4.831×10⁻⁴
```

Contoh hitungan:
```
T = 26°C (kolam renang), S = 0 PSU (air tawar):
ρ(26, 0) = 999.842594 + 1.766 - 6.149 + 1.759 - 0.449 + 0.012 = 996.78 kg/m³

T = 20°C, S = 0:
ρ(20, 0) = 998.20 kg/m³

T = 20°C, S = 35 PSU (air laut):
ρ(20, 35) = 1024.76 kg/m³
```

### B. Tekanan Hidrostatis:
```
P(z) = P_atm + ρ × g × z

Di kedalaman 0.8 m (kolam 26°C):
P = 101325 + 996.78 × 9.80665 × 0.8 = 101325 + 7817 = 109142 Pa = 1.091 bar
```

### C. Model Arus Air (Gauss-Markov Orde-1):

Arus air dimodelkan sebagai proses stokastik dengan temporal correlation:
```
d(turb)/dt = -(1/T_corr) × turb + σ × sqrt(2/T_corr) × w(t)

Di mana:
  T_corr = 4.0 s (waktu korelasi)
  σ = intensitas turbulensi (bervariasi per preset)
  w(t) = white noise Gaussian (Box-Muller)
```

### D. Preset Lingkungan:

| Preset | Kec. Arus | Turbulensi | Salinitas | Keterangan |
|:-------|:----------|:-----------|:----------|:-----------|
| `CALM` | 0.0 m/s | 0.002 m/s | 0 PSU | Air diam total |
| `POOL_CIRCULATION` | 0.0 m/s | 0.0 m/s | 0 PSU | Kolam indoor (default) |
| `MILD_CROSS_CURRENT` | 0.12 m/s | 0.03 m/s | 0 PSU | Arus lateral ringan |
| `STRONG_CURRENT` | 0.25 m/s | 0.06 m/s | 35 PSU | Arus laut kuat |
| `TURBULENT_WAVE` | 0.10 m/s | 0.08 m/s | 0 PSU | Gelombang turbulen |

> **Referensi Implementasi:** File `src/dt-core/EnvironmentModel.js`.

---

## 14. 🔧 Model Dinamika Thruster T200 BlueRobotics

### A. Tabel PWM → Thrust Interpolasi (16V / 4S LiPo):

Data dari official Blue Robotics Performance Data (Bollard Test, September 2019):

| PWM (μs) | Thrust (kgf) | RPM | Current (A) | Keterangan |
|:----------|:-------------|:----|:------------|:-----------|
| 1100 | -4.10 | -3200 | 22.0 | Full reverse |
| 1200 | -2.40 | -2400 | 12.0 | |
| 1300 | -0.95 | -1600 | 5.0 | |
| 1400 | -0.18 | -800 | 1.5 | |
| 1470 | 0.00 | 0 | 0.5 | **Deadband start** |
| 1500 | 0.00 | 0 | 0.5 | **Neutral** |
| 1530 | 0.00 | 0 | 0.5 | **Deadband end** |
| 1600 | +0.22 | +900 | 1.8 | |
| 1700 | +1.10 | +1700 | 5.5 | |
| 1800 | +2.70 | +2500 | 13.0 | |
| 1900 | +5.10 | +3600 | 25.0 | **Full forward** |

### B. Spesifikasi Thruster T200:
```
Diameter propeler: 76 mm (0.076 m)
Motor Kv: 540 RPM/V
Thrust maks maju: 5.1 kgf (50.0 N) @ 16V
Thrust maks mundur: 4.1 kgf (40.2 N) @ 16V
RPM maksimal: 3600 RPM @ 16V full forward
Deadband PWM: 1470 - 1530 μs
Time constant fisik: 0.42 detik
```

### C. Model Voltage Sag:
```
sagFactor = V_actual / V_nominal = V_bat / 16.0

Jika baterai 14.5V:
  sagFactor = 14.5 / 16.0 = 0.906
  Thrust efektif = 0.906 × nominal thrust
```

> **Referensi Implementasi:** File `src/services/SystemIdentificationEngine.js` (lookup table), `src/services/ThrusterDynamicsModel.js` (motor dynamics, voltage sag, deadband).

---

## 15. 📋 Tabel Komprehensif Seluruh Parameter & Satuan SI Terkalibrasi

| Nama Parameter | Notasi Simbol | Nilai Numerik | Satuan SI | Sumber Kalibrasi |
| :--- | :--- | :--- | :--- | :--- |
| **Massa Kering Kapal** | m | 11.5 | kg | Penimbangan digital darat |
| **Momen Inersia Roll** | Ixx | 0.12 | kg·m² | Ekstraksi CAD Mesh & URDF |
| **Momen Inersia Pitch**| Iyy | 0.22 | kg·m² | Ekstraksi CAD Mesh & URDF |
| **Momen Inersia Yaw**  | Izz | 0.24 | kg·m² | Ekstraksi CAD Mesh & URDF |
| **Dimensi Panjang** | L | 0.54 | m | Pengukuran fisik |
| **Dimensi Lebar** | W | 0.28 | m | Pengukuran fisik |
| **Dimensi Tinggi** | H | 0.24 | m | Pengukuran fisik |
| **Volume Benaman Air** | V | 0.01151 | m³ (11.51 L) | Uji benaman + ballast weights |
| **Densitas Air Kolam (26°C)** | ρ | 996.78 | kg/m³ | UNESCO 1980 / Kell 1975 |
| **Percepatan Gravitasi**| g | 9.80665 | m/s² | Konstanta geofisika lokal |
| **Pusat Gravitasi (CG)**| [xG, yG, zG] | [0.0, 0.0, 0.0] | meter | Asal kerangka bodi {b} |
| **Pusat Apung (CB)**   | [xB, yB, zB] | [0.0, 0.0, -0.025] | meter | Posisi busa apung atas |
| **Added Mass Surge**   | Xu_dot | 5.5 | kg | Empiris / Berg (2012) |
| **Added Mass Sway**    | Yv_dot | 12.7 | kg | Empiris / Wu (2018) |
| **Added Mass Heave**   | Zw_dot | 14.6 | kg | Empiris / Fossen (2021) |
| **Added Inertia Roll** | Kp_dot | 0.12 | kg·m² | Empiris |
| **Added Inertia Pitch** | Mq_dot | 0.12 | kg·m² | Empiris |
| **Added Inertia Yaw** | Nr_dot | 0.12 | kg·m² | Empiris |
| **Damping Linier Surge**| Xu | 4.03 | N·s/m | Uji deselerasi luncur kolam |
| **Damping Quad Surge** | Xuu | 18.18 | N·s²/m² | Uji kecepatan terminal kolam |
| **Damping Linier Sway** | Yv | 6.22 | N·s/m | Uji geser menyamping |
| **Damping Quad Sway**  | Yvv | 21.66 | N·s²/m² | Uji gerak lateral kolam |
| **Damping Linier Heave**| Zw | 5.18 | N·s/m | Uji selam vertikal kolam |
| **Damping Quad Heave** | Zww | 36.99 | N·s²/m² | Uji selam vertikal kolam |
| **Damping Linier Roll/Pitch/Yaw** | Kp, Mq, Nr | 0.07 | N·m·s/rad | Empiris |
| **Damping Quad Roll/Pitch/Yaw** | Kpp, Mqq, Nrr | 1.55 | N·m·s²/rad² | Empiris |
| **Gaya Dorong Maks Maju**| T_max_fwd | +50.0 | Newton (5.1 kgf) | Datasheet T200 @ 16V |
| **Gaya Dorong Maks Mundur**| T_max_rev | -40.2 | Newton (4.1 kgf) | Datasheet T200 @ 16V |
| **Konstanta Waktu Motor**| tau_m | 0.35 | detik | Step response test |
| **Time Constant Fisik** | tau_phys | 0.42 | detik | Bollard test dyno |
| **Tegangan Baterai**   | V_bat | 16.0 | Volt | LiPo 4S 10.000 mAh |
| **Deadband PWM Motor** | PWM_deadband | 1470 s.d. 1530 | mikrodetik (μs) | BlueRobotics Basic ESC |
| **Kecepatan Maks Surge** | u_max | 1.5 | m/s | Spesifikasi operasi |
| **Kecepatan Maks Sway** | v_max | 1.2 | m/s | Spesifikasi operasi |
| **Kecepatan Maks Heave** | w_max | 0.8 | m/s | Spesifikasi operasi |
| **Laju Yaw Maks** | r_max | 2.0 | rad/s | Spesifikasi operasi |
| **Kedalaman Maks** | z_max | 2.0 | m | Arena operasi kolam |
| **RLS Forgetting Factor** | λ | 0.985 | (dimensionless) | Tuning empiris |

---

## 16. 🚀 Panduan Menjalankan Sistem

### 1. Aplikasi Desktop Utama (Windows Native Digital Twin):
```bash
# Instalasi dependensi (pertama kali)
npm install

# Jalankan aplikasi desktop
npm run desktop
```
Atau double-click file: **`Launch_Desktop_App.bat`** (Membuka antarmuka 3D real-time dengan akselerasi GPU RTX 4050).

### 2. Menjalankan Simulasi ROS 2 (WSL2):
```bash
# Di dalam WSL2 Ubuntu 22.04:
cd simulator
./setup_wsl_gazebo.sh
```
Atau double-click file: **`Launch_Gazebo_WSL.bat`**.

### 3. Mode Development (Browser):
```bash
npm run dev
```
Akses di `http://localhost:5173` untuk mode pengembangan dengan hot-reload.

---

## 17. 📁 Struktur Kode Sumber & Arsitektur Perangkat Lunak

```
digitaltwin/
├── src/
│   ├── dt-core/                          ← INTI MESIN FISIKA DIGITAL TWIN
│   │   ├── VehicleConfig.js              ← Single Source of Truth parameter wahana
│   │   ├── HydrodynamicsEngine.js        ← 6-DOF Fossen dynamics (M, C, D, g, RK4)
│   │   ├── Kinematics.js                 ← Transformasi koordinat, kuaternion, Euler
│   │   ├── EnvironmentModel.js           ← UNESCO densitas air, arus Gauss-Markov
│   │   ├── StateEstimator.js             ← 15-state Extended Kalman Filter (EKF)
│   │   ├── PINNResidual.js               ← Physics-Informed Neural Network residual
│   │   ├── ParameterIdentifier.js        ← Online parameter identification
│   │   ├── ThrusterTwinEngine.js         ← Individual thruster digital twin
│   │   ├── ValidationEngine.js           ← Digital twin vs real data validation
│   │   ├── UncertaintyEstimator.js       ← Kovarians & confidence interval
│   │   ├── OODDetector.js                ← Out-of-Distribution detection
│   │   ├── DTState.js                    ← State management digital twin
│   │   ├── DTHealthScore.js              ← Health scoring & diagnostics
│   │   ├── DataLogger.js                 ← Logging & recording sessions
│   │   ├── ReplayEngine.js               ← Replay recorded sessions
│   │   ├── SyncManager.js                ← Real ↔ Virtual sync orchestrator
│   │   ├── PredictionAPI.js              ← Predictive analytics API
│   │   └── TelemetrySchema.js            ← Telemetry data schema definitions
│   │
│   ├── services/                         ← SERVICE LAYER & CONTROLLERS
│   │   ├── AUVMotionController.js        ← Closed-loop PID flight controller
│   │   ├── HydrodynamicsEngine.js        ← Simplified hydro (compatibility layer)
│   │   ├── SystemIdentificationEngine.js ← ARMAX/RLS online system identification
│   │   ├── ThrusterDynamicsModel.js      ← T200 motor dynamics, voltage sag, lag
│   │   ├── PidController.js              ← Generic PID controller implementation
│   │   ├── MockRosConnection.js          ← ROS2 simulator (SITL mode)
│   │   ├── RosConnection.js              ← Real ROS2 WebSocket bridge
│   │   ├── TopicSubscriber.js            ← ROS2 topic subscription manager
│   │   ├── TopicPublisher.js             ← ROS2 topic publisher
│   │   ├── SensorNoiseModel.js           ← Realistic sensor noise profiles
│   │   ├── SubseaCollisionEngine.js      ← Underwater collision detection
│   │   └── WebSerialManager.js           ← Web Serial USB (HIL thruster test)
│   │
│   ├── components/                       ← UI COMPONENTS
│   │   ├── 3d/                           ← Three.js 3D viewport & models
│   │   ├── dashboard/                    ← Telemetry dashboard panels
│   │   └── thruster-test/                ← HIL thruster test interface
│   │
│   ├── store/                            ← State management (Zustand)
│   ├── App.jsx                           ← Root React component
│   └── main.jsx                          ← Application entry point
│
├── electron/                             ← Electron desktop wrapper
│   └── main.cjs                          ← Electron main process
│
├── jetson/                               ← Jetson Nano ROS2 launch files
├── simulator/                            ← Gazebo simulation configs
├── tests/                                ← Unit & integration tests
├── public/                               ← Static assets & 3D models
│
├── Launch_Desktop_App.bat                ← Windows desktop launcher
├── Launch_Gazebo_WSL.bat                 ← WSL2 Gazebo launcher
├── package.json                          ← NPM dependencies
├── vite.config.js                        ← Vite build configuration
└── README.md                             ← Project overview
```

---

> **Referensi Utama:**
> - T. I. Fossen, *"Handbook of Marine Craft Hydrodynamics and Motion Control"*, 2nd ed., John Wiley & Sons, 2021.
> - Berg, V., *"Development and Commissioning of a DP System for ROV SF 30k"*, M.Sc. Thesis, NTNU, 2012.
> - Wu, G., *"Identification of Hydrodynamic Coefficients for an ROV"*, Journal of Ocean Engineering, 2018.
> - Blue Robotics, *"T200 Thruster Performance Data"*, https://bluerobotics.com, 2019.
> - UNESCO, *"International Equation of State of Seawater"*, Technical Papers in Marine Science No. 36, 1980.

---

Developed with ❤️ for **Amarine FILKOM Universitas Brawijaya**.
