# 📘 Panduan Komprehensif Matematika & Arsitektur Teknis Digital Twin AUV 6-DOF
## Untuk Presentasi Sidang Tugas Akhir / Skripsi, Publikasi Jurnal, dan Dokumentasi Teknis

Dokumen ini menyajikan formulasi matematis lengkap, penurunan persamaan (*derivations*), parameter numerik terkalibrasi, matriks alokasi thruster, fusi sensor *15-State Extended Kalman Filter* (EKF), identifikasi sistem *Recursive Least Squares* (RLS), hingga arsitektur integrasi *Hardware-in-the-Loop* (HIL) dan *Software-in-the-Loop* (SITL) ROS 2 untuk robot **Autonomous Underwater Vehicle (AUV) Amarine FILKOM Universitas Brawijaya**.

---

## 📑 Daftar Isi
1. [Arsitektur Sistem Digital Twin (HIL & SITL)](#1-arsitektur-sistem-digital-twin-hil--sitl)
2. [Sistem Koordinat & Kinematika 6-DOF](#2-sistem-koordinat--kinematika-6-dof)
3. [Dinamika Hidrodinamika Nonlinier Fossen (Kinetika 6-DOF)](#3-dinamika-hidrodinamika-nonlinier-fossen-kinetika-6-dof)
4. [Hidrostatis, Gaya Apung, dan Metasenter (*Positive Buoyancy Fail-Safe*)](#4-hidrostatis-gaya-apung-dan-metasenter-positive-buoyancy-fail-safe)
5. [Matriks Alokasi Thruster (Thruster Allocation Matrix - TAM) & Pemodelan Motor](#5-matriks-alokasi-thruster-thruster-allocation-matrix---tam--pemodelan-motor)
6. [Fusi Sensor Asinkron: 15-State Extended Kalman Filter (EKF)](#6-fusi-sensor-asinkron-15-state-extended-kalman-filter-ekf)
7. [Identifikasi Sistem Daring (Online SysID) & Adaptasi RLS](#7-identifikasi-sistem-daring-online-sysid--adaptasi-rls)
8. [Pengendali Gerak Closed-Loop PID dengan Kompensasi Feedforward](#8-pengendali-gerak-closed-loop-pid-dengan-kompensasi-feedforward)
9. [Integrasi Numerik Runge-Kutta Orde ke-4 (RK4) & Residual PINN](#9-integrasi-numerik-runge-kutta-orde-ke-4-rk4--residual-pinn)
10. [Metrik Kesehatan Digital Twin & Deteksi Out-of-Distribution (OOD)](#10-metrik-kesehatan-digital-twin--deteksi-out-of-distribution-ood)
11. [Daftar Simbol, Satuan, dan Parameter Fisik Terkalibrasi](#11-daftar-simbol-satuan-dan-parameter-fisik-terkalibrasi)
12. [Topik ROS 2 & Protokol Telemetri](#12-topik-ros-2--protokol-telemetri)

---

## 1. 🏗️ Arsitektur Sistem Digital Twin (HIL & SITL)

Digital Twin ini mengintegrasikan lapisan fisik robot (*Physical Twin*) dan lingkungan simulasi komputasional berkinerja tinggi (*Virtual Twin*) secara dwiarah (*bi-directional real-time telemetry*):

```
┌─────────────────────────────────────────────────────────────┐
│                 PHYSICAL TWIN (HARDWARE / HIL)              │
│  - Komputer Onboard: NVIDIA Jetson Nano / Orin Nano         │
│  - Mikrokontroler: STM32 / Arduino / Teensy (ESC & Sensors) │
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
│  │ 6. Subsea Raycasting Collision & Boundary Engine      │  │
│  │ 7. OOD & Sensor Mahalanobis Fault Detector            │  │
│  └───────────────────────────────────────────────────────┘  │
│                               │                             │
│                               ▼                             │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ 3D Rendering CAD Viewport (Three.js WebGL / RTX 4050) │  │
│  │ - Dynamic Ocean Shader & Caustics Surface             │  │
│  │ - 6-DOF Pose Sync, Thruster Vector Visualizer         │  │
│  │ - Live Sensor Uncertainty Ellipsoid & Particle Trails │  │
│  └───────────────────────────────────────────────────────┘  │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               │ Perintah Kontrol: /cmd_vel
                               │ Effort: /thruster_commands
                               ▼
┌─────────────────────────────────────────────────────────────┐
│            SITL SIMULATOR (Gazebo Garden / ROS 2 Humble)    │
│  - WSL2 Ubuntu 22.04 LTS (NVIDIA Container Toolkit)         │
│  - UUV Simulator / Buoyancy & Hydrodynamics Plugins         │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 🌐 Sistem Koordinat & Kinematika 6-DOF

Dalam pemodelan wahana bawah air 6 derajat kebebasan (*6 Degrees of Freedom*), digunakan dua kerangka acuan utama sesuai standar **SNAME (1950)** dan **Fossen (2021)**:

1. **Kerangka Acuan Inersia / Bumi (*North-East-Down - NED Frame*) $\{n\}$**:
   - $x_n$: Arah Utara (*North*) $[m]$
   - $y_n$: Arah Timur (*East*) $[m]$
   - $z_n$: Arah Bawah menuju pusat bumi / Kedalaman (*Down / Depth*) $[m]$
2. **Kerangka Acuan Bodi (*Body-Fixed Frame*) $\{b\}$**:
   - $x_b$: Sumbu longitudinal wahana, positif ke arah depan (*Surge*)
   - $y_b$: Sumbu transversal wahana, positif ke arah kanan/lambung kanan (*Sway*)
   - $z_b$: Sumbu vertikal wahana, positif ke arah bawah lunas (*Heave*)

```
Derajat Kebebasan (6-DOF):
1. Surge  (x) : Translasi maju/mundur       (kecepatan linier u)
2. Sway   (y) : Translasi geser kanan/kiri   (kecepatan linier v)
3. Heave  (z) : Translasi naik/turun         (kecepatan linier w)
4. Roll   (φ) : Rotasi guling sumbu-x        (kecepatan sudut p)
5. Pitch  (θ) : Rotasi angguk sumbu-y        (kecepatan sudut q)
6. Yaw    (ψ) : Rotasi geleng/haluan sumbu-z (kecepatan sudut r)
```

### Vektor Keadaan (*State Vectors*):
$$\boldsymbol{\eta} = \begin{bmatrix} \boldsymbol{\eta}_1 \\ \boldsymbol{\eta}_2 \end{bmatrix} = \begin{bmatrix} x \\ y \\ z \\ \phi \\ \theta \\ \psi \end{bmatrix} \in \mathbb{R}^6 \quad (\text{Posisi \& Orientasi Euler dalam } \{n\})$$

$$\boldsymbol{\nu} = \begin{bmatrix} \boldsymbol{\nu}_1 \\ \boldsymbol{\nu}_2 \end{bmatrix} = \begin{bmatrix} u \\ v \\ w \\ p \\ q \\ r \end{bmatrix} \in \mathbb{R}^6 \quad (\text{Kecepatan Linier \& Sudut dalam } \{b\})$$

$$\boldsymbol{\tau} = \begin{bmatrix} \boldsymbol{\tau}_1 \\ \boldsymbol{\tau}_2 \end{bmatrix} = \begin{bmatrix} X \\ Y \\ Z \\ K \\ M \\ N \end{bmatrix} \in \mathbb{R}^6 \quad (\text{Gaya \& Momen Generalisasi dalam } \{b\})$$

---

### Transformasi Kinematika Euler ($ZYX$ Convention):
Hubungan antara turunan posisi $\boldsymbol{\dot{\eta}}$ terhadap kecepatan bodi $\boldsymbol{\nu}$ adalah:

$$\boldsymbol{\dot{\eta}} = \boldsymbol{J}(\boldsymbol{\eta}_2) \boldsymbol{\nu} = \begin{bmatrix} \boldsymbol{R}_b^n(\boldsymbol{\eta}_2) & \boldsymbol{0}_{3 \times 3} \\ \boldsymbol{0}_{3 \times 3} & \boldsymbol{T}_\Theta(\boldsymbol{\eta}_2) \end{bmatrix} \begin{bmatrix} \boldsymbol{\nu}_1 \\ \boldsymbol{\nu}_2 \end{bmatrix}$$

#### 1. Matriks Rotasi Linier $\boldsymbol{R}_b^n(\phi, \theta, \psi) \in SO(3)$:
$$\boldsymbol{R}_b^n = \begin{bmatrix} 
\cos\psi\cos\theta & -\sin\psi\cos\phi + \cos\psi\sin\theta\sin\phi & \sin\psi\sin\phi + \cos\psi\sin\theta\cos\phi \\
\sin\psi\cos\theta & \cos\psi\cos\phi + \sin\psi\sin\theta\sin\phi & -\cos\psi\sin\phi + \sin\psi\sin\theta\cos\phi \\
-\sin\theta & \cos\theta\sin\phi & \cos\theta\cos\phi
\end{bmatrix}$$

Sifat ortogonal: $\boldsymbol{R}_b^n = (\boldsymbol{R}_n^b)^T \implies (\boldsymbol{R}_b^n)^{-1} = (\boldsymbol{R}_b^n)^T$.

#### 2. Matriks Transformasi Kecepatan Sudut $\boldsymbol{T}_\Theta(\phi, \theta)$:
$$\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} = \begin{bmatrix} 
1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\
0 & \cos\phi & -\sin\phi \\
0 & \frac{\sin\phi}{\cos\theta} & \frac{\cos\phi}{\cos\theta}
\end{bmatrix} \begin{bmatrix} p \\ q \\ r \end{bmatrix}$$

> ⚠️ **Catatan Singularitas**: Matriks $\boldsymbol{T}_\Theta$ mengalami *gimbal lock* saat $\theta = \pm 90^\circ$ ($\cos\theta = 0$). Oleh karena itu, Digital Twin AUV mengimplementasikan representasi **Unit Kuaternion** $\boldsymbol{q} = [q_w, q_x, q_y, q_z]^T$ untuk integrasi numerik non-singular:

$$\boldsymbol{\dot{q}} = \frac{1}{2} \boldsymbol{\Omega}(\boldsymbol{\nu}_2) \boldsymbol{q} = \frac{1}{2} \begin{bmatrix} 0 & -p & -q & -r \\ p & 0 & r & -q \\ q & -r & 0 & p \\ r & q & -p & 0 \end{bmatrix} \begin{bmatrix} q_w \\ q_x \\ q_y \\ q_z \end{bmatrix}$$

---

## 3. 🌊 Dinamika Hidrodinamika Nonlinier Fossen (Kinetika 6-DOF)

Persamaan gerak dinamika AUV nonlinier dinyatakan dalam persamaan Fossen (2021):

$$\boldsymbol{M} \boldsymbol{\dot{\nu}} + \boldsymbol{C}(\boldsymbol{\nu})\boldsymbol{\nu} + \boldsymbol{D}(\boldsymbol{\nu}_r)\boldsymbol{\nu}_r + \boldsymbol{g}(\boldsymbol{\eta}) = \boldsymbol{\tau}_{\text{thruster}} + \boldsymbol{\tau}_{\text{env}} + \boldsymbol{\tau}_{\text{residual}}$$

Di mana kecepatan relatif terhadap arus air $\boldsymbol{\nu}_c = [u_c, v_c, w_c, 0, 0, 0]^T$ adalah:
$$\boldsymbol{\nu}_r = \boldsymbol{\nu} - \boldsymbol{\nu}_c$$

---

### A. Matriks Massa Total ($\boldsymbol{M} = \boldsymbol{M}_{RB} + \boldsymbol{M}_A \in \mathbb{R}^{6 \times 6}$)

#### 1. Matriks Massa Bodi Kaku (*Rigid-Body Mass Matrix*) $\boldsymbol{M}_{RB}$:
Dengan massa kapal $m = 11.5\text{ kg}$, pusat massa $CG = [x_G, y_G, z_G]^T = [0, 0, 0]^T$, dan tensor inersia:
$$\boldsymbol{M}_{RB} = \begin{bmatrix}
m & 0 & 0 & 0 & m z_G & -m y_G \\
0 & m & 0 & -m z_G & 0 & m x_G \\
0 & 0 & m & m y_G & -m x_G & 0 \\
0 & -m z_G & m y_G & I_{xx} & -I_{xy} & -I_{xz} \\
m z_G & 0 & -m x_G & -I_{xy} & I_{yy} & -I_{yz} \\
-m y_G & m x_G & 0 & -I_{xz} & -I_{yz} & I_{zz}
\end{bmatrix}$$

Dengan substitusi parameter robot aktual ($I_{xx} = 0.12, I_{yy} = 0.22, I_{zz} = 0.24\text{ kg}\cdot\text{m}^2$ dan $I_{xy}=I_{xz}=I_{yz}=0$):
$$\boldsymbol{M}_{RB} = \text{diag}([11.5, \, 11.5, \, 11.5, \, 0.12, \, 0.22, \, 0.24])$$

#### 2. Matriks Massa Tambah Fluida (*Hydrodynamic Added Mass Matrix*) $\boldsymbol{M}_A$:
Fluida di sekitar bodi yang ikut berakselerasi menimbulkan gaya inersia hidrodinamika:
$$\boldsymbol{M}_A = -\text{diag}([X_{\dot{u}}, \, Y_{\dot{v}}, \, Z_{\dot{w}}, \, K_{\dot{p}}, \, M_{\dot{q}}, \, N_{\dot{r}}])$$

Nilai terkalibrasi (*empirical & towing tank identification*):
- $X_{\dot{u}} = -5.5\text{ kg}$ (Surge added mass)
- $Y_{\dot{v}} = -12.7\text{ kg}$ (Sway added mass - penampang samping lebih lebar)
- $Z_{\dot{w}} = -14.6\text{ kg}$ (Heave added mass - pelat datar rangka atas/bawah)
- $K_{\dot{p}} = -0.12\text{ kg}\cdot\text{m}^2$, $M_{\dot{q}} = -0.12\text{ kg}\cdot\text{m}^2$, $N_{\dot{r}} = -0.12\text{ kg}\cdot\text{m}^2$

#### 3. Matriks Massa Efektif Total $\boldsymbol{M} = \boldsymbol{M}_{RB} + \boldsymbol{M}_A$:
$$\boldsymbol{M} = \begin{bmatrix}
11.5 + 5.5 & 0 & 0 & 0 & 0 & 0 \\
0 & 11.5 + 12.7 & 0 & 0 & 0 & 0 \\
0 & 0 & 11.5 + 14.6 & 0 & 0 & 0 \\
0 & 0 & 0 & 0.12 + 0.12 & 0 & 0 \\
0 & 0 & 0 & 0 & 0.22 + 0.12 & 0 \\
0 & 0 & 0 & 0 & 0 & 0.24 + 0.12
\end{bmatrix} = \begin{bmatrix}
17.0 & 0 & 0 & 0 & 0 & 0 \\
0 & 24.2 & 0 & 0 & 0 & 0 \\
0 & 0 & 26.1 & 0 & 0 & 0 \\
0 & 0 & 0 & 0.24 & 0 & 0 \\
0 & 0 & 0 & 0 & 0.34 & 0 \\
0 & 0 & 0 & 0 & 0 & 0.36
\end{bmatrix}$$

Invers matriks massa $\boldsymbol{M}^{-1}$ dihitung untuk percepatan linier/sudut:
$$\boldsymbol{M}^{-1} = \text{diag}([0.05882, \, 0.04132, \, 0.03831, \, 4.16667, \, 2.94118, \, 2.77778])$$

---

### B. Matriks Coriolis & Sentripetal ($\boldsymbol{C}(\boldsymbol{\nu}) = \boldsymbol{C}_{RB}(\boldsymbol{\nu}) + \boldsymbol{C}_A(\boldsymbol{\nu}_r)$)

Matriks Coriolis dimodelkan menggunakan operator *skew-symmetric* $\boldsymbol{S}(\boldsymbol{a})$ di mana $\boldsymbol{S}(\boldsymbol{a})\boldsymbol{b} = \boldsymbol{a} \times \boldsymbol{b}$:
$$\boldsymbol{S}(\boldsymbol{a}) = \begin{bmatrix} 0 & -a_3 & a_2 \\ a_3 & 0 & -a_1 \\ -a_2 & a_1 & 0 \end{bmatrix}$$

Struktur matriks Coriolis $6 \times 6$:
$$\boldsymbol{C}(\boldsymbol{\nu}) = \begin{bmatrix} 
\boldsymbol{0}_{3 \times 3} & -\boldsymbol{S}(\boldsymbol{a}_{RB} + \boldsymbol{a}_A) \\
-\boldsymbol{S}(\boldsymbol{a}_{RB} + \boldsymbol{a}_A) & -\boldsymbol{S}(\boldsymbol{b}_{RB} + \boldsymbol{b}_A)
\end{bmatrix}$$

Dengan momentum linier & angular:
- $\boldsymbol{a}_{RB} = m \boldsymbol{\nu}_1 = [m u, \, m v, \, m w]^T$
- $\boldsymbol{b}_{RB} = \boldsymbol{I}_b \boldsymbol{\nu}_2 = [I_{xx} p, \, I_{yy} q, \, I_{zz} r]^T$
- $\boldsymbol{a}_A = [-X_{\dot{u}} u_r, \, -Y_{\dot{v}} v_r, \, -Z_{\dot{w}} w_r]^T$
- $\boldsymbol{b}_A = [-K_{\dot{p}} p, \, -M_{\dot{q}} q, \, -N_{\dot{r}} r]^T$

---

### C. Matriks Redaman Hidrodinamika Nonlinier ($\boldsymbol{D}(\boldsymbol{\nu}_r)$)

Redaman hidrodinamika merupakan penjumlahan dari redaman linier $\boldsymbol{D}_L$ (*skin friction & potential damping*) dan redaman kuadratik $\boldsymbol{D}_Q$ (*cross-flow drag & vortex shedding*):

$$\boldsymbol{D}(\boldsymbol{\nu}_r)\boldsymbol{\nu}_r = \boldsymbol{D}_L \boldsymbol{\nu}_r + \boldsymbol{D}_Q |\boldsymbol{\nu}_r| \boldsymbol{\nu}_r$$

Persamaan per sumbu:
$$\begin{aligned}
F_{\text{drag, surge}} &= -(X_u u_r + X_{u|u|} |u_r| u_r) \\
F_{\text{drag, sway}}  &= -(Y_v v_r + Y_{v|v|} |v_r| v_r) \\
F_{\text{drag, heave}} &= -(Z_w w_r + Z_{w|w|} |w_r| w_r) \\
M_{\text{drag, roll}}  &= -(K_p p + K_{p|p|} |p| p) \\
M_{\text{drag, pitch}} &= -(M_q q + M_{q|q|} |q| q) \\
M_{\text{drag, yaw}}   &= -(N_r r + N_{r|r|} |r| r)
\end{aligned}$$

#### Nilai Parameter Redaman Terkalibrasi:
| Koefisien | Nilai Linier ($\boldsymbol{D}_L$) | Koefisien | Nilai Kuadratik ($\boldsymbol{D}_Q$) | Satuan |
| :--- | :--- | :--- | :--- | :--- |
| $X_u$ | $4.03$ | $X_{u\|u\|}$ | $18.18$ | $\text{N}\cdot\text{s}/\text{m}$ / $\text{N}\cdot\text{s}^2/\text{m}^2$ |
| $Y_v$ | $6.22$ | $Y_{v\|v\|}$ | $21.66$ | $\text{N}\cdot\text{s}/\text{m}$ / $\text{N}\cdot\text{s}^2/\text{m}^2$ |
| $Z_w$ | $5.18$ | $Z_{w\|w\|}$ | $36.99$ | $\text{N}\cdot\text{s}/\text{m}$ / $\text{N}\cdot\text{s}^2/\text{m}^2$ |
| $K_p$ | $0.07$ | $K_{p\|p\|}$ | $1.55$ | $\text{N}\cdot\text{m}\cdot\text{s}/\text{rad}$ / $\text{N}\cdot\text{m}\cdot\text{s}^2/\text{rad}^2$ |
| $M_q$ | $0.07$ | $M_{q\|q\|}$ | $1.55$ | $\text{N}\cdot\text{m}\cdot\text{s}/\text{rad}$ / $\text{N}\cdot\text{m}\cdot\text{s}^2/\text{rad}^2$ |
| $N_r$ | $0.07$ | $N_{r\|r\|}$ | $1.55$ | $\text{N}\cdot\text{m}\cdot\text{s}/\text{rad}$ / $\text{N}\cdot\text{m}\cdot\text{s}^2/\text{rad}^2$ |

---

## 4. 🤿 Hidrostatis, Gaya Apung, dan Metasenter (*Positive Buoyancy Fail-Safe*)

### A. Prinsip Archimedes & Keseimbangan Vertikal

Gaya berat wahana ($W$) dan gaya apung hidrostatis ($B$):
$$W = m \cdot g = 11.5 \times 9.80665 = 112.7765\text{ N}$$
$$B = \rho_{\text{water}} \cdot g \cdot \nabla = 998.2 \times 9.80665 \times 0.01225 = 119.914\text{ N}$$

Di mana:
- $\rho_{\text{water}} = 998.2\text{ kg/m}^3$ (densitas air tawar kolam uji pada suhu 20°C)
- $\nabla = 0.01225\text{ m}^3$ (volume benaman air wahana terpasang bodi & spons apung)
- $g = 9.80665\text{ m/s}^2$ (percepatan gravitasi)

$$\Delta F_{\text{net}} = B - W = 119.914\text{ N} - 112.7765\text{ N} = +7.1375\text{ N} \quad (\approx +727.8\text{ gram-force})$$

> 💡 **Karakteristik Fail-Safe Positif**: Gaya apung neto $+7.14\text{ N}$ memastikan wahana **selalu melayang naik ke permukaan secara pasif** apabila terjadi mati daya baterai, putus komunikasi, atau *failsafe disarm*.

---

### B. Vektor Gaya Pemulih Hidrostatis $\boldsymbol{g}(\boldsymbol{\eta}) \in \mathbb{R}^6$

Letak Pusat Massa $CG = [x_G, y_G, z_G]^T = [0, 0, 0]^T$ dan Pusat Apung $CB = [x_B, y_B, z_B]^T = [0, 0, -0.025]^T\text{ m}$ (dalam sistem NED, $z_B < z_G$ berarti $CB$ berada $25\text{ mm}$ **di atas** $CG$).

$$\boldsymbol{g}(\boldsymbol{\eta}) = \begin{bmatrix}
(W - B)\sin\theta \\
-(W - B)\cos\theta\sin\phi \\
-(W - B)\cos\theta\cos\phi \\
-(y_G W - y_B B)\cos\theta\cos\phi + (z_G W - z_B B)\cos\theta\sin\phi \\
(z_G W - z_B B)\sin\theta + (x_G W - x_B B)\cos\theta\cos\phi \\
-(x_G W - x_B B)\cos\theta\sin\phi - (y_G W - y_B B)\sin\theta
\end{bmatrix}$$

Karena $x_G = y_G = x_B = y_B = 0$ dan $z_G = 0$:
$$\boldsymbol{g}(\boldsymbol{\eta}) = \begin{bmatrix}
(W - B)\sin\theta \\
-(W - B)\cos\theta\sin\phi \\
-(W - B)\cos\theta\cos\phi \\
-z_B B \cos\theta\sin\phi \\
-z_B B \sin\theta \\
0
\end{bmatrix} = \begin{bmatrix}
-7.138 \sin\theta \\
7.138 \cos\theta\sin\phi \\
7.138 \cos\theta\cos\phi \\
2.998 \cos\theta\sin\phi \\
2.998 \sin\theta \\
0
\end{bmatrix} \quad [\text{N dan N}\cdot\text{m}]$$

Momen pemulih (*righting moment*) pada Roll ($K_g = 2.998 \sin\phi$) dan Pitch ($M_g = 2.998 \sin\theta$) bekerja sebagai **pegas mekanis alami fluida** yang secara otomatis menstabilkan orientasi datar AUV.

---

## 5. 🌀 Matriks Alokasi Thruster (Thruster Allocation Matrix - TAM) & Pemodelan Motor

Robot AUV dilengkapi dengan **6 unit BlueRobotics T200 Brushless Thruster** yang dikendalikan oleh ESC bi-directional:
- **$T_1, T_2, T_3, T_4$ (Horizontal Vectoring)**: Dipasang pada 4 sudut dengan kemiringan $\alpha = 45^\circ$ ($\pi/4\text{ rad}$) untuk mengendalikan $X$ (*Surge*), $Y$ (*Sway*), dan $N$ (*Yaw*).
- **$T_5, T_6$ (Vertical Heave)**: Dipasang secara vertikal pada garis simetri depan dan belakang ($x_{T5} = +0.18\text{ m}, x_{T6} = -0.18\text{ m}$) untuk mengendalikan $Z$ (*Heave*) dan $M$ (*Pitch*).

```
               ▲ Depan (+x_b)
               │
    T1 (45°) ┌───┐ T2 (-45°)
      \      │   │      /
       \  ┌──┴───┴──┐  /
          │  [T5]   │     Lebar = 0.28 m
          │ (Heave) │     Panjang = 0.54 m
          │         │
          │  [T6]   │
       /  └──┬───┬──┘  \
      /      │   │      \
    T3 (135°)└───┘ T4 (-135°)
               │
               ▼ Belakang (-x_b)
```

### Koordinat Posisi dan Vektor Arah Thruster dalam $\{b\}$:
| Thruster | Posisi $[x, y, z]$ (m) | Vektor Arah Gaya $[\hat{d}_x, \hat{d}_y, \hat{d}_z]$ | Peran Gerakan |
| :--- | :--- | :--- | :--- |
| **$T_1$** (Depan-Kiri) | $[+0.14, \, +0.11, \, 0.0]$ | $[\cos 45^\circ, \, -\sin 45^\circ, \, 0] = [0.7071, \, -0.7071, \, 0]$ | Surge (+), Sway (-), Yaw (+) |
| **$T_2$** (Depan-Kanan)| $[+0.14, \, -0.11, \, 0.0]$ | $[\cos 45^\circ, \, +\sin 45^\circ, \, 0] = [0.7071, \, +0.7071, \, 0]$ | Surge (+), Sway (+), Yaw (-) |
| **$T_3$** (Belakang-Kiri)| $[-0.14, \, +0.11, \, 0.0]$ | $[\cos 45^\circ, \, +\sin 45^\circ, \, 0] = [0.7071, \, +0.7071, \, 0]$ | Surge (+), Sway (+), Yaw (+) |
| **$T_4$** (Belakang-Kanan)| $[-0.14, \, -0.11, \, 0.0]$ | $[\cos 45^\circ, \, -\sin 45^\circ, \, 0] = [0.7071, \, -0.7071, \, 0]$ | Surge (+), Sway (-), Yaw (-) |
| **$T_5$** (Vertikal Depan)| $[+0.18, \, 0.0, \, 0.0]$ | $[0, \, 0, \, -1.0]$ | Heave (Turun saat $T>0$), Pitch (-) |
| **$T_6$** (Vertikal Belakang)| $[-0.18, \, 0.0, \, 0.0]$ | $[0, \, 0, \, -1.0]$ | Heave (Turun saat $T>0$), Pitch (+) |

---

### A. Matriks Konfigurasi Alokasi Thruster $\boldsymbol{T}_{\text{alloc}} \in \mathbb{R}^{6 \times 6}$

Persamaan pemetaan gaya dorong individu $\boldsymbol{u}_T = [T_1, T_2, T_3, T_4, T_5, T_6]^T$ ke gaya dan momen generalisasi bodi $\boldsymbol{\tau} = [X, Y, Z, K, M, N]^T$:

$$\boldsymbol{\tau} = \boldsymbol{T}_{\text{alloc}} \boldsymbol{u}_T$$

Di mana baris-baris $\boldsymbol{T}_{\text{alloc}}$ dihitung dari perkalian silang lengan momen $\boldsymbol{r}_i \times \boldsymbol{d}_i$:

$$\boldsymbol{T}_{\text{alloc}} = \begin{bmatrix}
\cos 45^\circ & \cos 45^\circ & \cos 45^\circ & \cos 45^\circ & 0 & 0 \\
-\sin 45^\circ & \sin 45^\circ & \sin 45^\circ & -\sin 45^\circ & 0 & 0 \\
0 & 0 & 0 & 0 & -1 & -1 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0.18 & -0.18 \\
L_{\text{arm1}} & -L_{\text{arm2}} & L_{\text{arm3}} & -L_{\text{arm4}} & 0 & 0
\end{bmatrix}$$

Dengan lengan momen Yaw $L_{\text{arm}} = x_i \sin 45^\circ + y_i \cos 45^\circ$:
$$L_{\text{arm}} = 0.14 \times \frac{\sqrt{2}}{2} + 0.11 \times \frac{\sqrt{2}}{2} = 0.09899 + 0.07778 = 0.17678\text{ m}$$

Substitusi numerik lengkap:
$$\boldsymbol{T}_{\text{alloc}} = \begin{bmatrix}
0.7071 & 0.7071 & 0.7071 & 0.7071 & 0 & 0 \\
-0.7071 & 0.7071 & 0.7071 & -0.7071 & 0 & 0 \\
0 & 0 & 0 & 0 & -1.0 & -1.0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0.18 & -0.18 \\
0.1768 & -0.1768 & 0.1768 & -0.1768 & 0 & 0
\end{bmatrix}$$

---

### B. Inversi Alokasi Kontrol (Moore-Penrose Pseudo-Inverse)

Untuk menghasilkan gaya yang diminta oleh pengendali PID $\boldsymbol{\tau}_{\text{cmd}} = [F_x, F_y, F_z, 0, M_y, M_z]^T$, gaya dorong tiap thruster dihitung melalui pseudo-inverse:

$$\boldsymbol{u}_T = \boldsymbol{T}_{\text{alloc}}^\dagger \boldsymbol{\tau}_{\text{cmd}} = \boldsymbol{T}_{\text{alloc}}^T (\boldsymbol{T}_{\text{alloc}} \boldsymbol{T}_{\text{alloc}}^T)^{-1} \boldsymbol{\tau}_{\text{cmd}}$$

Solusi eksplisit closed-form per thruster:
$$\begin{aligned}
T_1 &= \frac{F_x}{4\cos 45^\circ} - \frac{F_y}{4\sin 45^\circ} + \frac{M_z}{4 L_{\text{arm}}} = 0.3536 F_x - 0.3536 F_y + 1.4141 M_z \\
T_2 &= \frac{F_x}{4\cos 45^\circ} + \frac{F_y}{4\sin 45^\circ} - \frac{M_z}{4 L_{\text{arm}}} = 0.3536 F_x + 0.3536 F_y - 1.4141 M_z \\
T_3 &= \frac{F_x}{4\cos 45^\circ} + \frac{F_y}{4\sin 45^\circ} + \frac{M_z}{4 L_{\text{arm}}} = 0.3536 F_x + 0.3536 F_y + 1.4141 M_z \\
T_4 &= \frac{F_x}{4\cos 45^\circ} - \frac{F_y}{4\sin 45^\circ} - \frac{M_z}{4 L_{\text{arm}}} = 0.3536 F_x - 0.3536 F_y - 1.4141 M_z \\
T_5 &= -\frac{F_z}{2} + \frac{M_y}{2 \times 0.18} = -0.5 F_z + 2.7778 M_y \\
T_6 &= -\frac{F_z}{2} - \frac{M_y}{2 \times 0.18} = -0.5 F_z - 2.7778 M_y
\end{aligned}$$

---

### C. Pemodelan Dinamika Aktuator & Konversi PWM-Thrust

#### 1. Dinamika Respons Motor Orde Pertama (*First-Order Lag*):
$$\dot{T}_i(t) = \frac{1}{\tau_m} (T_{\text{target}, i}(t) - T_i(t))$$
Dengan konstanta waktu motor terkalibrasi $\tau_m = 0.35\text{ detik}$.

#### 2. Karakteristik Kurva Thrust vs PWM (Tegangan Nominal 16.0V 4S LiPo):
- Deadband PWM: $[1470\mu\text{s}, \, 1530\mu\text{s}]$ ($T = 0\text{ N}$)
- Maju ($PWM \ge 1530\mu\text{s}$): $T_{\text{max}} = +50.0\text{ N}$ (+5.1 kgf pada $1900\mu\text{s}$)
  $$T_i(PWM) = c_{f1} (PWM - 1500) + c_{f2} (PWM - 1500)^2$$
- Mundur ($PWM \le 1470\mu\text{s}$): $T_{\text{min}} = -40.2\text{ N}$ (-4.1 kgf pada $1100\mu\text{s}$)
  $$T_i(PWM) = c_{r1} (PWM - 1500) - c_{r2} (PWM - 1500)^2$$

---

## 6. 🛰️ Fusi Sensor Asinkron: 15-State Extended Kalman Filter (EKF)

Digital Twin menggabungkan multi-sensor asinkron pada frekuensi berbeda (*IMU 100 Hz, Barometer Depth 20 Hz, DVL 10 Hz, Kompas 20 Hz*) menggunakan EKF 15-state:

### Vektor Keadaan EKF ($\boldsymbol{x} \in \mathbb{R}^{15}$):
$$\boldsymbol{x} = \begin{bmatrix}
\boldsymbol{p}^n_{3 \times 1} \\
\boldsymbol{v}^b_{3 \times 1} \\
\boldsymbol{\Theta}_{3 \times 1} \\
\boldsymbol{b}_{a, 3 \times 1} \\
\boldsymbol{b}_{g, 3 \times 1}
\end{bmatrix} = \begin{bmatrix}
[x, \, y, \, z]^T & (\text{Posisi NED, m}) \\
[u, \, v, \, w]^T & (\text{Kecepatan bodi, m/s}) \\
[\phi, \, \theta, \, \psi]^T & (\text{Orientasi Euler, rad}) \\
[b_{ax}, \, b_{ay}, \, b_{az}]^T & (\text{Bias Akselerometer, m/s}^2) \\
[b_{gx}, \, b_{gy}, \, b_{gz}]^T & (\text{Bias Giroskop, rad/s})
\end{bmatrix}$$

---

### A. Langkah Prediksi (*Time-Update Step*)
Dijalankan setiap interval IMU $\Delta t = 0.01\text{ s}$ dengan input akselerasi spesifik terukur $\boldsymbol{f}_m = [a_{x,m}, a_{y,m}, a_{z,m}]^T$ dan laju putar $\boldsymbol{\omega}_m = [g_{x,m}, g_{y,m}, g_{z,m}]^T$:

$$\hat{\boldsymbol{x}}_{k|k-1} = \hat{\boldsymbol{x}}_{k-1|k-1} + \boldsymbol{f}(\hat{\boldsymbol{x}}_{k-1|k-1}, \boldsymbol{u}_k) \Delta t$$

Persamaan diferensial model keadaan:
$$\begin{aligned}
\boldsymbol{\dot{p}}^n &= \boldsymbol{R}_b^n(\boldsymbol{\Theta}) \boldsymbol{v}^b \\
\boldsymbol{\dot{v}}^b &= (\boldsymbol{f}_m - \boldsymbol{b}_a) - \boldsymbol{S}(\boldsymbol{\omega}_m - \boldsymbol{b}_g)\boldsymbol{v}^b + \boldsymbol{R}_n^b \boldsymbol{g}^n \\
\boldsymbol{\dot{\Theta}} &= \boldsymbol{T}_\Theta(\boldsymbol{\Theta}) (\boldsymbol{\omega}_m - \boldsymbol{b}_g) \\
\boldsymbol{\dot{b}}_a &= \boldsymbol{w}_{ba} \quad (\text{Random Walk Drift}, \, \sigma_{ba}^2 = 10^{-5}) \\
\boldsymbol{\dot{b}}_g &= \boldsymbol{w}_{bg} \quad (\text{Random Walk Drift}, \, \sigma_{bg}^2 = 10^{-6})
\end{aligned}$$

#### Propagasi Matriks Kovarians Ketidakpastian:
$$\boldsymbol{P}_{k|k-1} = \boldsymbol{\Phi}_k \boldsymbol{P}_{k-1|k-1} \boldsymbol{\Phi}_k^T + \boldsymbol{Q}_k$$
Di mana $\boldsymbol{\Phi}_k \approx \boldsymbol{I}_{15} + \boldsymbol{F}_k \Delta t$ dengan Jacobian $\boldsymbol{F}_k = \left. \frac{\partial \boldsymbol{f}}{\partial \boldsymbol{x}} \right|_{\hat{\boldsymbol{x}}}$.

---

### B. Langkah Koreksi (*Measurement-Update Step*) dengan Mahalanobis Gating
Ketika pembacaan sensor $\boldsymbol{z}_k$ tiba:

1. **Inovasi / Residual Pengukuran**:
   $$\tilde{\boldsymbol{y}}_k = \boldsymbol{z}_k - \boldsymbol{h}(\hat{\boldsymbol{x}}_{k|k-1})$$
2. **Kovarians Inovasi**:
   $$\boldsymbol{S}_k = \boldsymbol{H}_k \boldsymbol{P}_{k|k-1} \boldsymbol{H}_k^T + \boldsymbol{R}_k$$
3. **Uji Validasi Outlier Chi-Square ($\chi^2$-Gating)**:
   $$d_M^2 = \tilde{\boldsymbol{y}}_k^T \boldsymbol{S}_k^{-1} \tilde{\boldsymbol{y}}_k \le \gamma_{\text{threshold}}$$
   *(Jika $d_M^2 > 9.0$ untuk sensor 1D atau $> 14.16$ untuk sensor 3D, data ditolak sebagai outlier / spike noise).*
4. **Kalman Gain Optimal**:
   $$\boldsymbol{K}_k = \boldsymbol{P}_{k|k-1} \boldsymbol{H}_k^T \boldsymbol{S}_k^{-1}$$
5. **Pembaruan Keadaan & Kovarians**:
   $$\hat{\boldsymbol{x}}_{k|k} = \hat{\boldsymbol{x}}_{k|k-1} + \boldsymbol{K}_k \tilde{\boldsymbol{y}}_k$$
   $$\boldsymbol{P}_{k|k} = (\boldsymbol{I}_{15} - \boldsymbol{K}_k \boldsymbol{H}_k) \boldsymbol{P}_{k|k-1} (\boldsymbol{I}_{15} - \boldsymbol{K}_k \boldsymbol{H}_k)^T + \boldsymbol{K}_k \boldsymbol{R}_k \boldsymbol{K}_k^T \quad (\text{Joseph Form})$$

---

## 7. 🧠 Identifikasi Sistem Daring (Online SysID) & Adaptasi RLS

Untuk mengantisipasi perubahan massa bodi (misal penambahan manipulator, kamera payload baru) atau hambatan arus air tak terduga, Digital Twin menjalankan estimasi parameter adaptif berbasis model polinomial diskrit **ARMAX (*AutoRegressive Moving Average with eXogenous inputs*)**:

$$A(q^{-1}) y(t) = B(q^{-1}) u(t - d) + C(q^{-1}) e(t)$$

Dalam bentuk representasi regresi linier:
$$y(t) = \boldsymbol{\varphi}^T(t) \boldsymbol{\theta}_0 + e(t)$$

- **Vektor Regresor**:
  $$\boldsymbol{\varphi}(t) = [-y(t-1), \, -y(t-2), \, u(t-1), \, u(t-2)]^T$$
- **Vektor Parameter Dinamis**:
  $$\boldsymbol{\theta}(t) = [a_1, \, a_2, \, b_1, \, b_2]^T$$

### Algoritma Recursive Least Squares (RLS) dengan Faktor Pembobotan Eksponensial (*Forgetting Factor* $\lambda$):

1. **Perhitungan Gain Adaptif $\boldsymbol{K}(t) \in \mathbb{R}^4$**:
   $$\boldsymbol{K}(t) = \frac{\boldsymbol{P}(t-1) \boldsymbol{\varphi}(t)}{\lambda + \boldsymbol{\varphi}^T(t) \boldsymbol{P}(t-1) \boldsymbol{\varphi}(t)}$$

2. **Perhitungan Error Prediksi A Priori $\varepsilon(t)$**:
   $$\varepsilon(t) = y(t) - \boldsymbol{\varphi}^T(t) \hat{\boldsymbol{\theta}}(t-1)$$

3. **Pembaruan Estimasi Parameter $\hat{\boldsymbol{\theta}}(t)$**:
   $$\hat{\boldsymbol{\theta}}(t) = \hat{\boldsymbol{\theta}}(t-1) + \boldsymbol{K}(t) \varepsilon(t)$$

4. **Pembaruan Matriks Kovarians Parameter $\boldsymbol{P}(t)$**:
   $$\boldsymbol{P}(t) = \frac{1}{\lambda} \left[ \boldsymbol{I}_4 - \boldsymbol{K}(t) \boldsymbol{\varphi}^T(t) \right] \boldsymbol{P}(t-1)$$

> 🔬 **Parameter Tuning**: Nilai forgetting factor dipilih $\lambda = 0.985$, memberikan jendela memori efektif $N_{\text{eff}} \approx \frac{1}{1 - \lambda} = 66.7\text{ sampel}$ ($1.33\text{ detik}$ pada sampling $50\text{ Hz}$).

---

## 8. 🎯 Pengendali Gerak Closed-Loop PID dengan Kompensasi Feedforward

Pengendali gerak AUV mengimplementasikan arsitektur **PID Independen per Sumbu Aktif (Surge, Sway, Heave, Yaw)** dengan proteksi *anti-windup integrator clamping*, filter derivatif orde-1, dan feedforward hidrostatis:

$$u_k = u_{\text{FF}} + K_p e_k + K_i \int e(\tau) d\tau + K_d \frac{d e_k}{dt}$$

### A. Pengendali Kedalaman (*Depth / Heave Controller*):
Karena gaya apung alami wahana selalu mendorong ke atas ($\Delta F_{\text{net}} = +7.14\text{ N}$), ditambahkan suku **Feedforward Buoyancy Cancellation**:

$$F_{z, \text{cmd}} = -(W - B) + K_{p,z} (z_{\text{target}} - z) + K_{i,z} \sum (z_{\text{target}} - z)\Delta t - K_{d,z} w$$

Dengan kompensasi $F_{\text{FF}} = -7.14\text{ N}$, thruster vertikal $T_5, T_6$ langsung memberikan dorongan tahanan bawah tanpa harus menunggu error integral menumpuk.

### B. Matriks Gain Pengendali PID Terkalibrasi:
| Derajat Kebebasan | $K_p$ | $K_i$ | $K_d$ | Batas Output Saturation | Anti-Windup Limit |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Surge ($u$)** | $25.0\text{ N/(m/s)}$ | $4.0\text{ N/m}$ | $12.0\text{ N}\cdot\text{s/m}$ | $\pm 80.0\text{ N}$ | $\pm 20.0\text{ N}$ |
| **Sway ($v$)** | $22.0\text{ N/(m/s)}$ | $3.5\text{ N/m}$ | $10.0\text{ N}\cdot\text{s/m}$ | $\pm 60.0\text{ N}$ | $\pm 15.0\text{ N}$ |
| **Heave ($z$)** | $45.0\text{ N/m}$ | $8.0\text{ N/(m}\cdot\text{s)}$ | $24.0\text{ N}\cdot\text{s/m}$ | $\pm 80.0\text{ N}$ | $\pm 25.0\text{ N}$ |
| **Yaw ($\psi$)** | $18.0\text{ N}\cdot\text{m/rad}$ | $2.5\text{ N}\cdot\text{m/(rad}\cdot\text{s)}$ | $9.5\text{ N}\cdot\text{m}\cdot\text{s/rad}$ | $\pm 20.0\text{ N}\cdot\text{m}$ | $\pm 5.0\text{ N}\cdot\text{m}$ |

---

## 9. ⚙️ Integrasi Numerik Runge-Kutta Orde ke-4 (RK4) & Residual PINN

Digital Twin menyelesaikan sistem persamaan diferensial hidrodinamika nonlinier menggunakan algoritma **Runge-Kutta 4th-Order (RK4)**:

$$\boldsymbol{\dot{\nu}} = \boldsymbol{f}(\boldsymbol{\eta}, \boldsymbol{\nu}, \boldsymbol{\tau}) = \boldsymbol{M}^{-1} \left[ \boldsymbol{\tau}_{\text{thrust}} + \boldsymbol{\tau}_{\text{PINN}} - \boldsymbol{C}(\boldsymbol{\nu})\boldsymbol{\nu} - \boldsymbol{D}(\boldsymbol{\nu}_r)\boldsymbol{\nu}_r - \boldsymbol{g}(\boldsymbol{\eta}) \right]$$

### Skema Komputasi RK4 ($\Delta t = 0.02\text{ s}$ / 50 Hz Engine):
$$\begin{aligned}
\boldsymbol{k}_1 &= \boldsymbol{f}(t_n, \, \boldsymbol{\nu}_n) \\
\boldsymbol{k}_2 &= \boldsymbol{f}\left(t_n + \frac{\Delta t}{2}, \, \boldsymbol{\nu}_n + \frac{\Delta t}{2} \boldsymbol{k}_1\right) \\
\boldsymbol{k}_3 &= \boldsymbol{f}\left(t_n + \frac{\Delta t}{2}, \, \boldsymbol{\nu}_n + \frac{\Delta t}{2} \boldsymbol{k}_2\right) \\
\boldsymbol{k}_4 &= \boldsymbol{f}\left(t_n + \Delta t, \, \boldsymbol{\nu}_n + \Delta t \boldsymbol{k}_3\right) \\
\boldsymbol{\nu}_{n+1} &= \boldsymbol{\nu}_n + \frac{\Delta t}{6} \left( \boldsymbol{k}_1 + 2\boldsymbol{k}_2 + 2\boldsymbol{k}_3 + \boldsymbol{k}_4 \right)
\end{aligned}$$

### Integrasi Fisika Terbantu AI (*Physics-Informed Neural Network - PINN*):
Model fisika ideal Fossen dilengkapi dengan jaringan neural residual $\mathcal{N}_{\boldsymbol{\theta}_{\text{NN}}}$ untuk memprediksi dinamika hidrodinamika yang tidak termodelkan (*unmodeled thruster-hull interactions & wave turbulence*):

$$\boldsymbol{\tau}_{\text{PINN}} = \mathcal{N}_{\boldsymbol{\theta}_{\text{NN}}}(\boldsymbol{\nu}, \boldsymbol{u}_T, \text{depth})$$

Fungsi rugi-rugi (*Loss Function*) saat pelatihan model PINN:
$$\mathcal{L}(\boldsymbol{\theta}_{\text{NN}}) = \underbrace{\frac{1}{N} \sum_{i=1}^N \|\boldsymbol{\nu}_{\text{real}} - \hat{\boldsymbol{\nu}}\|^2}_{\text{Data Loss}} + \lambda_{\text{physics}} \underbrace{\left\| \boldsymbol{M}\boldsymbol{\dot{\nu}} + \boldsymbol{C}\boldsymbol{\nu} + \boldsymbol{D}\boldsymbol{\nu} + \boldsymbol{g} - \boldsymbol{\tau}_{\text{total}} \right\|^2}_{\text{Physics-Informed Conservation of Momentum Loss}}$$

---

## 10. 📊 Metrik Kesehatan Digital Twin & Deteksi Out-of-Distribution (OOD)

Untuk mengevaluasi keandalan sinkronisasi antara robot fisik dan replika virtual, dihitung **Digital Twin Composite Health Score ($H \in [0, 100\%]$)**:

$$H = 100 \times \left( w_1 S_{\text{sync}} + w_2 S_{\text{residual}} + w_3 S_{\text{battery}} + w_4 S_{\text{temp}} \right)$$

Di mana:
1. **Skor Sinkronisasi Posisi ($S_{\text{sync}}$)**:
   $$S_{\text{sync}} = \exp\left( -\frac{\|\boldsymbol{p}_{\text{real}} - \boldsymbol{p}_{\text{twin}}\|^2}{2 \sigma_p^2} \right)$$
2. **Jarak Mahalanobis Inovasi EKF ($d_M^2$)**:
   $$d_M^2 = (\boldsymbol{z}_{\text{meas}} - \hat{\boldsymbol{z}})^T \boldsymbol{S}^{-1} (\boldsymbol{z}_{\text{meas}} - \hat{\boldsymbol{z}})$$
   $$S_{\text{residual}} = \max\left(0, \, 1 - \frac{d_M^2}{\chi^2_{\text{critical}}}\right)$$
3. **Kesehatan Baterai & Daya**:
   $$S_{\text{battery}} = \frac{V_{\text{actual}} - V_{\text{cutoff}}}{V_{\text{nominal}} - V_{\text{cutoff}}} \quad (V_{\text{cutoff}} = 13.8\text{V}, \, V_{\text{nominal}} = 16.8\text{V})$$

---

## 11. 📋 Daftar Simbol, Satuan, dan Parameter Fisik Terkalibrasi

| Parameter | Notasi Simbol | Nilai Numerik | Satuan | Sumber / Metode Kalibrasi |
| :--- | :--- | :--- | :--- | :--- |
| **Massa Total Kering** | $m$ | $11.5$ | $\text{kg}$ | Timbangan digital presisi |
| **Inersia Roll** | $I_{xx}$ | $0.12$ | $\text{kg}\cdot\text{m}^2$ | Ekstraksi CAD Mesh & URDF |
| **Inersia Pitch** | $I_{yy}$ | $0.22$ | $\text{kg}\cdot\text{m}^2$ | Ekstraksi CAD Mesh & URDF |
| **Inersia Yaw** | $I_{zz}$ | $0.24$ | $\text{kg}\cdot\text{m}^2$ | Ekstraksi CAD Mesh & URDF |
| **Volume Benaman** | $\nabla$ | $0.01225$ | $\text{m}^3$ | Uji benaman Archimedes |
| **Densitas Air** | $\rho$ | $998.2$ | $\text{kg/m}^3$ | Refraktometer / Termometer kolam |
| **Gravitasi** | $g$ | $9.80665$ | $\text{m/s}^2$ | Konstanta geofisika Malang |
| **Pusat Massa (CG)** | $[x_G, y_G, z_G]$ | $[0.0, 0.0, 0.0]$ | $\text{m}$ | Geometric Origin $\{b\}$ |
| **Pusat Apung (CB)** | $[x_B, y_B, z_B]$ | $[0.0, 0.0, -0.025]$ | $\text{m}$ | Pengaturan busa apung atas |
| **Added Mass Surge** | $-X_{\dot{u}}$ | $5.5$ | $\text{kg}$ | SysID Empiris Berg (2012) |
| **Added Mass Sway** | $-Y_{\dot{v}}$ | $12.7$ | $\text{kg}$ | SysID Empiris Wu (2018) |
| **Added Mass Heave** | $-Z_{\dot{w}}$ | $14.6$ | $\text{kg}$ | SysID Empiris Fossen (2021) |
| **Damping Linier Surge** | $X_u$ | $4.03$ | $\text{N}\cdot\text{s/m}$ | Uji deselerasi luncur kolam |
| **Damping Kuadratik Surge** | $X_{u\|u\|}$ | $18.18$ | $\text{N}\cdot\text{s}^2/\text{m}^2$ | Uji gaya dorong vs kec. terminal |
| **Damping Linier Sway** | $Y_v$ | $6.22$ | $\text{N}\cdot\text{s/m}$ | Uji deselerasi sway |
| **Damping Kuadratik Sway** | $Y_{v\|v\|}$ | $21.66$ | $\text{N}\cdot\text{s}^2/\text{m}^2$ | Uji gerak lateral kolam |
| **Damping Linier Heave** | $Z_w$ | $5.18$ | $\text{N}\cdot\text{s/m}$ | Uji selam vertikal kolam |
| **Damping Kuadratik Heave**| $Z_{w\|w\|}$ | $36.99$ | $\text{N}\cdot\text{s}^2/\text{m}^2$ | Uji selam vertikal kolam |
| **Thrust Maksimum Fwd** | $T_{\text{max, fwd}}$ | $50.0$ | $\text{N}$ | Datasheet T200 @ 16V |
| **Thrust Maksimum Rev** | $T_{\text{max, rev}}$ | $40.2$ | $\text{N}$ | Datasheet T200 @ 16V |
| **Konstanta Waktu ESC** | $\tau_m$ | $0.35$ | $\text{s}$ | Step response dyno test |

---

## 12. 📡 Topik ROS 2 & Protokol Telemetri

| Topik ROS 2 | Tipe Pesan | Frekuensi | Deskripsi Sinyal |
| :--- | :--- | :--- | :--- |
| `/odom` | `nav_msgs/msg/Odometry` | 50 Hz | Posisi 3D $[x,y,z]$, kuaternion $[q_w,q_x,q_y,q_z]$, kecepatan linier & angular |
| `/imu/data` | `sensor_msgs/msg/Imu` | 100 Hz | Akselerasi 3-sumbu $[a_x, a_y, a_z]$ dan laju giroskop $[p, q, r]$ |
| `/depth` | `sensor_msgs/msg/FluidPressure` | 20 Hz | Tekanan absolut air $[Pa]$ dan konversi kedalaman $[m]$ |
| `/dvl/range` | `sensor_msgs/msg/Range` | 10 Hz | Ketinggian akustik dari dasar kolam (*Altitude*) |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | 1 Hz | Tegangan LiPo 4S ($V$), arus beban ($A$), dan sisa daya ($\%$) |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 50 Hz | Target kecepatan 4-DOF (*Surge, Sway, Heave, Yaw*) dari pengendali |
| `/thruster_commands` | `std_msgs/msg/Float64MultiArray` | 50 Hz | Vektor perintah PWM / gaya 6-thruster $[T_1 \dots T_6]$ |
| `/thruster_outputs` | `std_msgs/msg/Float64MultiArray` | 50 Hz | Umpan balik aktual daya dorong dari hardware ESC Jetson |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 30 Hz | Kompresi JPEG video stream kamera depan subsea |
| `/dt/health_score` | `std_msgs/msg/Float64` | 10 Hz | Skor kesehatan sinkronisasi Digital Twin ($0-100\%$) |

---

## 13. 🚀 Panduan Peluncuran Sistem

1. **Aplikasi Desktop Utama (Windows Native Digital Twin)**:
   - Jalankan file **`Launch_Desktop_App.bat`** (Membuka antarmuka visualisasi 3D Three.js + WebGL dengan akselerasi GPU RTX 4050).
2. **Backend Simulasi ROS 2 (Docker Environment)**:
   - Jalankan file **`simulator/start_simulation.bat`** (Memulai container ROS 2 Humble bridge dan node hidrodinamika).
3. **Simulasi 3D Gazebo di WSL2**:
   - Jalankan file **`Launch_Gazebo_WSL.bat`** (Membuka simulator fisika Gazebo native via WSLg).
