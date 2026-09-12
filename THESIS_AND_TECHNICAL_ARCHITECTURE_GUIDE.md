# 📘 Panduan Lengkap & Mudah Dipahami: Arsitektur dan Matematika Digital Twin AUV 6-DOF
## Khusus Persiapan Sidang Skripsi / Tugas Akhir & Presentasi Teknis
**Robot Autonomous Underwater Vehicle (AUV) Amarine — FILKOM Universitas Brawijaya**

---

## 🎯 Tujuan Dokumen Ini
Dokumen ini dibuat dengan **bahasa yang jelas, runtut, dan mudah dipahami** (disertai analogi dunia nyata, contoh hitungan angka langsung, dan bocoran tanya-jawab sidang), sehingga kamu bisa menjelaskan proyek Digital Twin ini ke dosen penguji dengan percaya diri tanpa pusing dengan rumus yang rumit!

---

## 📑 Daftar Isi Cepat
1. [Konsep Dasar: Apa itu Digital Twin AUV?](#1-konsep-dasar-apa-itu-digital-twin-auv)
2. [Sistem Koordinat & Gerakan 6-DOF (Bahasa Sederhana)](#2-sistem-koordinat--gerakan-6-dof-bahasa-sederhana)
3. [Fisika Gerak Kapal Bawah Air (Persamaan Fossen Dijelaskan Mudah)](#3-fisika-gerak-kapal-bawah-air-persamaan-fossen-dijelaskan-mudah)
4. [Hitungan Gaya Apung & Kenapa Kapal Tidak Akan Tenggelam (Fail-Safe)](#4-hitungan-gaya-apung--kenapa-kapal-tidak-akan-tenggelam-fail-safe)
5. [Rahasia 6 Thruster: Cara Kapal Maju, Geser, dan Muter (TAM)](#5-rahasia-6-thruster-cara-kapal-maju-geser-dan-muter-tam)
6. [Fusi Sensor EKF 15-State: Menggabungkan Sensor yang Berisik](#6-fusi-sensor-ekf-15-state-menggabungkan-sensor-yang-berisik)
7. [Adaptasi Otomatis RLS (SysID): Kapal yang Menyesuaikan Diri](#7-adaptasi-otomatis-rls-sysid-kapal-yang-menyesuaikan-diri)
8. [Pengendali PID: Cara Menjaga Posisi dan Kedalaman Stabil](#8-pengendali-pid-cara-menjaga-posisi-dan-kedalaman-stabil)
9. [🔥 Bocoran Pertanyaan Dosen Penguji Sidang & Cara Menjawabnya](#9--bocoran-pertanyaan-dosen-penguji-sidang--cara-menjawabnya)
10. [Tabel Angka & Parameter Fisik Robot Asli](#10-tabel-angka--parameter-fisik-robot-asli)

---

## 1. 💡 Konsep Dasar: Apa itu Digital Twin AUV?

### Analogi Sederhana:
Bayangkan kamu punya robot kapal selam asli di dalam kolam, dan di laptop kamu ada **kembaran digitalnya (avatar 3D)**. Apapun yang terjadi pada kapal asli (bergerak, miring, terdorong arus air), kembaran di laptop akan bergerak persis sama secara *real-time*. Bahkan laptop bisa menghitung jika ada sensor yang rusak atau arus air yang tiba-tiba kencang.

```
┌────────────────────────────────┐                 ┌─────────────────────────────────┐
│       KAPAL ASLI (Hardware)    │                 │      DIGITAL TWIN (Software)    │
│  - Komputer: Jetson Nano       │  Kirim Sensor   │  - Tampilan 3D (Three.js WebGL) │
│  - Sensor: IMU, Barometer, DVL │ ──────────────> │  - Mesin Fisika (Fossen 6-DOF)  │
│  - 6 Motor Thruster T200       │  (WebSocket)    │  - Fusi Sensor (EKF 15-State)   │
│  - Baterai LiPo 16V            │ <────────────── │  - Kendali Cerdas PID + RLS     │
└────────────────────────────────┘  Kirim Perintah └─────────────────────────────────┘
```

---

## 2. 🧭 Sistem Koordinat & Gerakan 6-DOF (Bahasa Sederhana)

Kapal selam bergerak bebas di dalam air dengan **6 Derajat Kebebasan (6-DOF / Degrees of Freedom)**:
- **3 Gerakan Geser (Translasi)**:
  1. **Surge ($u$)**: Gerak maju / mundur (sepanjang sumbu $X$).
  2. **Sway ($v$)**: Gerak geser samping kanan / kiri (sepanjang sumbu $Y$).
  3. **Heave ($w$)**: Gerak menyelam turun / naik ke permukaan (sepanjang sumbu $Z$).
- **3 Gerakan Putar (Rotasi)**:
  4. **Roll ($p, \phi$)**: Gerak miring guling ke kanan / kiri.
  5. **Pitch ($q, \theta$)**: Gerak hidung kapal mendongak ke atas / menukik ke bawah.
  6. **Yaw ($r, \psi$)**: Gerak belok haluan ke kanan / kiri (seperti setir mobil).

### Aturan Kerangka Koordinat (NED - North East Down):
- Sumbu $X$ = Menghadap ke depan (Utara).
- Sumbu $Y$ = Menghadap ke kanan kapal (Timur).
- Sumbu $Z$ = Menghadap **ke bawah** (semakin dalam kapal menyelam, nilai $Z$ semakin positif).

---

## 3. 🌊 Fisika Gerak Kapal Bawah Air (Persamaan Fossen Dijelaskan Mudah)

Persamaan utama gerak kapal bawah air mengikuti hukum fisika Fossen:

$$
\boldsymbol{M} \boldsymbol{\dot{\nu}} + \boldsymbol{C}(\boldsymbol{\nu})\boldsymbol{\nu} + \boldsymbol{D}(\boldsymbol{\nu})\boldsymbol{\nu} + \boldsymbol{g}(\boldsymbol{\eta}) = \boldsymbol{\tau}
$$

Jangan takut dengan simbolnya! Ini sebenarnya adalah **Hukum II Newton ($F = m \cdot a$)** versi kapal selam:

| Simbol | Nama Fisika | Penjelasan Bahasa Manusia |
| :--- | :--- | :--- |
| $\boldsymbol{M} \boldsymbol{\dot{\nu}}$ | **Inersia Total** | Beban berat kapal ($11.5\text{ kg}$) + **air yang ikut terseret** saat kapal bergerak (*Added Mass*). |
| $\boldsymbol{C}(\boldsymbol{\nu})\boldsymbol{\nu}$ | **Gaya Coriolis** | Gaya sentrifugal/berputar saat kapal sedang belok sambil maju. |
| $\boldsymbol{D}(\boldsymbol{\nu})\boldsymbol{\nu}$ | **Hambatan Air (*Drag*)** | Gesekan air yang menahan laju kapal (makin cepat kapal melaju, gesekan air makin besar kuadratik). |
| $\boldsymbol{g}(\boldsymbol{\eta})$ | **Gaya Apung & Gravitasi** | Efek gaya Archimedes dan gravitasi bumi yang menjaga kapal tetap tegak. |
| $\boldsymbol{\tau}$ | **Gaya Dorong Thruster** | Total gaya dorong dorongan dari 6 motor propeller T200. |

---

### Contoh Hitungan Nyata: Mengapa Massa Air (*Added Mass*) Dihitung?

Saat kapal melaju maju (*Surge*), kapal tidak hanya menggerakkan bodi seberat $11.5\text{ kg}$, tetapi bagian depan kapal juga harus mendorong air seberat $5.5\text{ kg}$ di depannya.
- Berat bodi kering: $m = 11.5\text{ kg}$
- Massa air terseret maju: $X_{\dot{u}} = 5.5\text{ kg}$
- **Massa efektif total maju**:
$$
M_{\text{surge}} = 11.5 + 5.5 = 17.0\text{ kg}
$$

Saat kapal bergerak geser samping (*Sway*), bodi samping lebih lebar sehingga menyeret air lebih banyak ($12.7\text{ kg}$):
$$
M_{\text{sway}} = 11.5 + 12.7 = 24.2\text{ kg}
$$

> 💡 **Inti untuk Sidang**: Inilah alasan kenapa kapal selam butuh tenaga dorong lebih besar untuk geser ke samping dibanding maju lurus!

---

## 4. 🤿 Hitungan Gaya Apung & Kenapa Kapal Tidak Akan Tenggelam (Fail-Safe)

Robot AUV dirancang dengan prinsip **Gaya Apung Positif Alami (*Positive Buoyancy*)**.

### Langkah Hitungan Angka:
1. **Berat Kapal di Udara ($W$)**:
   - Massa kapal: $m = 11.5\text{ kg}$
   - Gravitasi: $g = 9.807\text{ m/s}^2$
   $$
   W = 11.5 \times 9.807 = 112.78\text{ Newton}
   $$

2. **Gaya Angkat Air Archimedes ($B$)**:
   - Densitas air kolam: $\rho = 998.2\text{ kg/m}^3$
   - Volume total kapal: $\nabla = 0.01225\text{ m}^3$ (12.25 liter)
   $$
   B = \rho \times g \times \nabla = 998.2 \times 9.807 \times 0.01225 = 119.91\text{ Newton}
   $$

3. **Gaya Bersih ke Atas ($\Delta F$)**:
   $$
   \Delta F = B - W = 119.91 - 112.78 = +7.13\text{ Newton} \quad (\approx +727\text{ gram-force})
   $$

### 🛡️ Fitur Keselamatan (Fail-Safe):
Karena $B > W$ sebesar $+7.13\text{ N}$, maka jika baterai habis atau kabel putus:
- Motor vertikal otomatis mati ($T_5 = 0, T_6 = 0$).
- Kapal **secara otomatis mengapung naik sendiri ke permukaan air** tanpa perlu daya listrik! Kapal tidak akan pernah tenggelam dan hilang di dasar kolam.

---

## 5. 🌀 Rahasia 6 Thruster: Cara Kapal Maju, Geser, dan Muter (TAM)

Kapal menggunakan 6 unit motor BlueRobotics T200:
- **4 Motor Horizontal ($T_1, T_2, T_3, T_4$)** dipasang di 4 sudut miring $45^\circ$.
- **2 Motor Vertikal ($T_5, T_6$)** dipasang tegak lurus di bagian depan dan belakang.

```
                  ▲ Depan (+X)
                  │
      T1 (45°)  ┌───┐  T2 (-45°)
        \       │   │       /
         \   ┌──┴───┴──┐   /
             │  [T5]   │     Lebar = 0.28 m
             │ (Heave) │     Panjang = 0.54 m
             │         │
             │  [T6]   │
         /   └──┬───┬──┘   \
        /       │   │       \
      T3 (135°) └───┘  T4 (-135°)
                  │
                  ▼ Belakang (-X)
```

### Cara Kerja Pembagian Tenaga Motor (Thruster Allocation):
1. **Maju Lurus (*Surge Forward*)**:
   Keempat motor sudut ($T_1, T_2, T_3, T_4$) menyala maju bersamaan. Karena miring $45^\circ$, komponen gaya sampingnya saling menghilangkan, menyisakan gaya dorong murni ke depan.
2. **Geser Samping Kanan (*Sway Right*)**:
   Motor $T_2$ dan $T_3$ dorong maju, $T_1$ dan $T_4$ dorong mundur. Hasilnya kapal bergeser ke kanan tanpa memutar badan.
3. **Belok Putar Haluan (*Yaw Turn*)**:
   Motor sisi kiri ($T_1, T_3$) dorong maju, motor sisi kanan ($T_2, T_4$) dorong mundur (seperti memutar roda tank).
4. **Menyelam Turun (*Dive Down*)**:
   Dua motor vertikal ($T_5, T_6$) mendorong air ke atas, menekan kapal turun melawan gaya apung alami.

### Rumus Perhitungan Daya Tiap Motor:
Jika pengendali meminta gaya maju $F_x$, gaya geser $F_y$, gaya selam $F_z$, dan momen putar $M_z$:
- $T_1 = 0.3536 F_x - 0.3536 F_y + 1.4141 M_z$
- $T_2 = 0.3536 F_x + 0.3536 F_y - 1.4141 M_z$
- $T_3 = 0.3536 F_x + 0.3536 F_y + 1.4141 M_z$
- $T_4 = 0.3536 F_x - 0.3536 F_y - 1.4141 M_z$
- $T_5 = -0.5 F_z + 2.7778 M_y$
- $T_6 = -0.5 F_z - 2.7778 M_y$

---

## 6. 🛰️ Fusi Sensor EKF 15-State: Menggabungkan Sensor yang Berisik

### Masalah di Bawah Air:
- Sensor IMU (akselerometer & giroskop) sangat cepat ($100\text{ Hz}$), tapi nilainya **gampang ngaco/drifting** lama-lama.
- Sensor Barometer kedalaman akurat, tapi lambat ($20\text{ Hz}$) dan ada gelombang air.
- Sensor DVL (kecepatan dasar kolam) lambat ($10\text{ Hz}$).

### Solusi Digital Twin (EKF 15-State):
Extended Kalman Filter (EKF) bertindak seperti **hakim penengah pintar**:
1. EKF memprediksi posisi kapal setiap milidetik memakai model fisika.
2. Saat data sensor barometer atau DVL masuk, EKF mengoreksi tebakan posisi tersebut.
3. EKF secara otomatis menghitung dan membuang bias error sensor sehingga kapal tidak pernah tersesat.

---

## 7. 🧠 Adaptasi Otomatis RLS (SysID): Kapal yang Menyesuaikan Diri

### Mengapa butuh RLS (*Recursive Least Squares*)?
Jika kapal diberi beban kamera baru atau arus air kolam tiba-tiba bertambah kencang, berat dan hambatan kapal berubah. Algoritma RLS di Digital Twin **mempelajari parameter baru ini secara otomatis dalam hitungan 1 detik ($\lambda = 0.985$)** tanpa perlu mematikan program!

---

## 8. 🎯 Pengendali PID: Cara Menjaga Posisi dan Kedalaman Stabil

Pengendali PID menghitung tenaga motor agar kapal tetap di target:
1. **$P$ (Proportional)**: Tenaga dorong sebanding dengan seberapa jauh jarak kapal dari target.
2. **$I$ (Integral)**: Menghapus error sisa yang belum tuntas.
3. **$D$ (Derivative)**: Memberikan efek rem lembut agar kapal tidak kebablasan (*overshoot*).

### Rahasia Penahan Kedalaman (*Feedforward Heave*):
Karena kapal punya gaya apung alami $+7.14\text{ N}$, pengendali kedalaman langsung memberi daya dasar awal sebesar $-7.14\text{ N}$ (*Feedforward*) sehingga motor vertikal langsung menahan kapal stabil di kedalaman target (misal $0.8\text{ meter}$) tanpa goyang naik turun!

---

## 9. 🔥 Bocoran Pertanyaan Dosen Penguji Sidang & Cara Menjawabnya

Berikut adalah daftar pertanyaan yang paling sering ditanyakan dosen penguji saat sidang skripsi beserta **jawaban tepat dan percaya diri**:

#### ❓ Pertanyaan 1: *"Apa bedanya Digital Twin ini dengan simulasi 3D biasa di Gazebo atau MATLAB?"*
> **Jawaban Kamu**:
> *"Perbedaannya ada pada **koneksi dua arah (bi-directional synchronization) dan adaptasi real-time**, Pak/Bu. Simulasi biasa hanya menjalankan model matematika statis. Sedangkan Digital Twin kami terhubung langsung ke sensor hardware robot asli melalui ROS2 WebSocket. Jika robot asli berbelok atau mengalami arus air di kolam, Digital Twin langsung menyinkronkan posisinya, mengestimasi bias sensor via EKF, dan memperbarui parameter hidrodinamika menggunakan adaptasi online RLS."*

#### ❓ Pertanyaan 2: *"Kenapa konfigurasi thruster horizontal dipasang miring 45 derajat, kenapa tidak lurus saja?"*
> **Jawaban Kamu**:
> *"Dengan posisi miring 45 derajat pada 4 sudut (vektorisasi TAM), robot mendapatkan kemampuan **Holonomic / Omnidirectional** pada bidang datar. Artinya robot bisa maju-mundur ($Surge$), geser kanan-kiri ($Sway$), dan berputar ($Yaw$) secara simultan hanya dengan 4 motor tanpa memerlukan sirip kemudi mekanik."*

#### ❓ Pertanyaan 3: *"Bagaimana kamu memastikan kapal tidak tenggelam jika terjadi sistem error atau baterai habis?"*
> **Jawaban Kamu**:
> *"Kapal kami dirancang dengan prinsip **Positive Buoyancy Fail-Safe**, di mana gaya apung fluida Archimedes ($B = 119.91\text{ N}$) sengaja dibuat lebih besar dari berat total kapal ($W = 112.78\text{ N}$). Ketika terjadi error atau daya putus, gaya neto sebesar $+7.14\text{ N}$ (setara +728 gram) akan secara alami dan pasif mengangkat kapal naik ke permukaan air."*

#### ❓ Pertanyaan 4: *"Apa fungsi EKF 15-State di sistem ini?"*
> **Jawaban Kamu**:
> *"EKF 15-state bertugas melakukan **fusi sensor multi-rate asinkron** antara IMU (100 Hz), sensor kedalaman (20 Hz), dan DVL (10 Hz). Selain mengestimasi posisi dan orientasi 6-DOF, EKF mengestimasi 6 parameter bias drift sensor secara online sehingga akumulasi error navigasi dapat dieliminasi."*

---

## 10. 📋 Tabel Angka & Parameter Fisik Robot Asli

| Nama Parameter | Simbol | Nilai Nyata | Satuan | Keterangan Praktis |
| :--- | :--- | :--- | :--- | :--- |
| **Massa Total Kering** | $m$ | $11.5$ | $\text{kg}$ | Ditimbang saat kering |
| **Panjang x Lebar x Tinggi** | $L \times W \times H$ | $0.54 \times 0.28 \times 0.24$ | $\text{meter}$ | Dimensi bodi rangka |
| **Volume Air Dipindahkan** | $\nabla$ | $0.01225$ | $\text{m}^3$ ($12.25\text{ liter}$) | Volume total benaman air |
| **Gaya Berat Kapal** | $W$ | $112.78$ | $\text{Newton}$ | Berat ke bawah akibat gravitasi |
| **Gaya Apung Archimedes** | $B$ | $119.91$ | $\text{Newton}$ | Gaya dorong ke atas oleh air |
| **Gaya Apung Bersih** | $\Delta F$ | $+7.14$ | $\text{Newton}$ ($+728\text{ g}$) | Kelebihan gaya apung pasif |
| **Massa Tambah Maju** | $X_{\dot{u}}$ | $5.5$ | $\text{kg}$ | Beban air terseret saat maju |
| **Massa Tambah Samping**| $Y_{\dot{v}}$ | $12.7$ | $\text{kg}$ | Beban air terseret saat geser |
| **Massa Tambah Vertikal**| $Z_{\dot{w}}$ | $14.6$ | $\text{kg}$ | Beban air terseret saat menyelam |
| **Daya Dorong Maks Motor** | $T_{\text{max}}$ | $50.0$ | $\text{Newton}$ ($5.1\text{ kgf}$) | Per motor T200 pada 16V |
| **Tegangan Baterai** | $V_{\text{bat}}$ | $16.0$ | $\text{Volt}$ | Baterai LiPo 4S |

---

## 11. 🚀 Cara Menjalankan Program (Demo Sidang)

1. **Buka Aplikasi Desktop Digital Twin**:
   - Klik ganda file: **`Launch_Desktop_App.bat`** (Membuka antarmuka 3D interaktif).
2. **Jalankan Simulator ROS 2**:
   - Klik ganda file: **`simulator/start_simulation.bat`**.
3. **Jalankan Simulasi Gazebo (Opsional)**:
   - Klik ganda file: **`Launch_Gazebo_WSL.bat`**.
