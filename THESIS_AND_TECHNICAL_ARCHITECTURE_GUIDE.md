# 📘 Panduan Lengkap & Penjelasan Hitungan: Digital Twin AUV 6-DOF
## Khusus Persiapan Sidang Skripsi / Tugas Akhir & Presentasi Teknis
**Robot Autonomous Underwater Vehicle (AUV) Amarine — FILKOM Universitas Brawijaya**

---

## 🎯 Panduan Membaca
Dokumen ini disusun menggunakan **bahasa Indonesia yang jelas dan bersih tanpa kode rumus rumit/tanda dolar**. Semua hitungan fisika, rumus gerak, dan logika sistem ditulis langsung dengan angka nyata agar mudah dipahami, dihafal, dan dijelaskan saat sidang tugas akhir.

---

## 📑 Daftar Isi
1. [Konsep Dasar: Apa itu Digital Twin AUV?](#1-konsep-dasar-apa-itu-digital-twin-auv)
2. [Sistem Gerak 6 Derajat Kebebasan (6-DOF)](#2-sistem-gerak-6-derajat-kebebasan-6-dof)
3. [Perhitungan Fisika Gerak Kapal (Hukum Fossen)](#3-perhitungan-fisika-gerak-kapal-hukum-fossen)
4. [Perhitungan Gaya Apung & Kenapa Kapal Anti-Tenggelam (Fail-Safe)](#4-perhitungan-gaya-apung--kenapa-kapal-anti-tenggelam-fail-safe)
5. [Perhitungan 6 Motor Thruster: Cara Kapal Maju, Geser & Muter](#5-perhitungan-6-motor-thruster-cara-kapal-maju-geser--muter)
6. [Fusi Sensor Cerdas EKF 15-State](#6-fusi-sensor-cerdas-ekf-15-state)
7. [Adaptasi Otomatis RLS saat Beban / Arus Air Berubah](#7-adaptasi-otomatis-rls-saat-beban--arus-air-berubah)
8. [Pengendali PID Kedalaman & Posisi](#8-pengendali-pid-kedalaman--posisi)
9. [🔥 Bocoran Pertanyaan Dosen Penguji Sidang & Cara Menjawabnya](#9--bocoran-pertanyaan-dosen-penguji-sidang--cara-menjawabnya)
10. [Rangkuman Lengkap Data & Parameter Robot](#10-rangkuman-lengkap-data--parameter-robot)

---

## 1. 💡 Konsep Dasar: Apa itu Digital Twin AUV?

### Analogi Sederhana:
Digital Twin adalah **replika digital (kembaran 3D di laptop)** yang terhubung secara langsung dengan **robot kapal selam fisik asli di kolam**.

- **Kapal Asli di Kolam**: Membawa sensor IMU, sensor kedalaman, kamera, dan 6 motor thruster. Kapal mengirimkan data gerak ke laptop melalui jaringan komunikasi ROS2 WebSocket (port 9090).
- **Digital Twin di Laptop**: Menerima data sensor secara langsung (50 kali per detik), menggerakkan model 3D secara persis sama, menghitung hambatan air, mendeteksi jika ada sensor yang rusak, dan mengirimkan perintah kendali otomatis ke kapal asli.

```
┌────────────────────────────────┐                 ┌─────────────────────────────────┐
│     KAPAL ASLI DI KOLAM        │                 │    DIGITAL TWIN DI LAPTOP       │
│  - Komputer Onboard: Jetson    │  Kirim Sensor   │  - Tampilan 3D (Three.js WebGL) │
│  - Sensor: IMU, Barometer, DVL │ ──────────────> │  - Simulasi Fisika 6-DOF (RK4)  │
│  - 6 Motor Thruster T200       │  (WebSocket)    │  - Fusi Sensor (EKF 15-State)   │
│  - Baterai LiPo 16 Volt        │ <────────────── │  - Kendali Cerdas PID + RLS     │
└────────────────────────────────┘  Kirim Perintah └─────────────────────────────────┘
```

---

## 2. 🧭 Sistem Gerak 6 Derajat Kebebasan (6-DOF)

Kapal selam bergerak di dalam air dengan **6 macam gerakan (6 Degrees of Freedom)**:

### A. 3 Gerakan Geser Lurus (Translasi):
1. **Surge (Maju / Mundur)**: Gerak lurus ke depan atau ke belakang (sumbu X). Kecepatannya disimbolkan dengan **u** (satuan meter/detik).
2. **Sway (Geser Kanan / Kiri)**: Gerak menyamping ke kanan atau ke kiri tanpa memutar badan (sumbu Y). Kecepatannya disimbolkan dengan **v** (satuan meter/detik).
3. **Heave (Menyelam / Naik)**: Gerak turun ke dasar kolam atau naik ke permukaan air (sumbu Z). Kecepatannya disimbolkan dengan **w** (satuan meter/detik).

### B. 3 Gerakan Putar (Rotasi):
4. **Roll (Guling)**: Gerakan badan kapal miring ke samping kanan atau kiri (sumbu putar X). Sudutnya disimbolkan dengan **phi (φ)**.
5. **Pitch (Angguk)**: Gerakan moncong depan kapal mendongak ke atas atau menukik ke bawah (sumbu putar Y). Sudutnya disimbolkan dengan **theta (θ)**.
6. **Yaw (Belok Haluan)**: Gerakan kapal memutar haluan ke kanan atau ke kiri seperti setir mobil (sumbu putar Z). Sudutnya disimbolkan dengan **psi (ψ)**.

### Aturan Arah Sumbu (Standar Maritim NED - North East Down):
- **Sumbu X** = Menghadap ke depan kapal.
- **Sumbu Y** = Menghadap ke lambung kanan kapal.
- **Sumbu Z** = Menghadap **ke bawah** (semakin dalam kapal menyelam, nilai kedalaman Z semakin bertambah positif).

---

## 3. 🌊 Perhitungan Fisika Gerak Kapal (Hukum Fossen)

Gerak kapal selam di dalam air dipengaruhi oleh 5 gaya utama:

**Gaya Dorong Motor = Inersia Total + Gaya Putar Coriolis + Gesekan Hambatan Air + Gaya Apung / Gravitasi**

Dalam bahasa fisika teknik maritim (Fossen):
`M * Percepatan + C * Kecepatan + D * Kecepatan + Gaya_Pemulih = Gaya_Motor`

---

### Hitungan Nyata: Mengapa Ada Massa Tambah Air (Added Mass)?

Ketika kapal bergerak di darat, kapal hanya menggerakkan berat badannya sendiri seberat **11.5 kg**.
Tetapi ketika kapal melaju di dalam air, air di sekitar bodi kapal ikut terdorong dan terseret. Beban air yang ikut bergerak ini disebut **Massa Tambah (Added Mass)**.

#### 1. Saat Kapal Maju Lurus (Surge):
- Massa bodi kering kapal = **11.5 kg**
- Beban air yang ikut terdorong di depan = **5.5 kg**
- **Total Beban Massa Maju = 11.5 kg + 5.5 kg = 17.0 kg**

#### 2. Saat Kapal Geser ke Samping (Sway):
Karena badan samping kapal lebih lebar daripada moncong depan, air yang harus disingkirkan jauh lebih banyak:
- Massa bodi kering kapal = **11.5 kg**
- Beban air yang terseret di samping = **12.7 kg**
- **Total Beban Massa Geser Samping = 11.5 kg + 12.7 kg = 24.2 kg**

#### 3. Saat Kapal Menyelam Turun (Heave):
Pelat atas dan bawah kapal sangat luas sehingga menahan banyak air:
- Massa bodi kering kapal = **11.5 kg**
- Beban air yang tertahan di atas/bawah = **14.6 kg**
- **Total Beban Massa Menyelam = 11.5 kg + 14.6 kg = 26.1 kg**

> 💡 **Kesimpulan untuk Sidang**:
> Kapal butuh tenaga motor lebih besar untuk geser ke samping (beban 24.2 kg) dan menyelam (beban 26.1 kg) dibandingkan saat melaju lurus ke depan (beban 17.0 kg).

---

### Hitungan Hambatan Gesekan Air (Hydrodynamic Drag):

Hambatan air bertambah sangat cepat saat kapal melaju lebih kencang (mengikuti hukum kuadratik):
- **Gaya Gesek Maju = (4.03 * Kecepatan) + (18.18 * Kecepatan * Kecepatan)**
- **Contoh**: Jika kapal melaju maju dengan kecepatan **0.5 meter/detik**:
  - Gesekan linier = 4.03 * 0.5 = 2.015 Newton
  - Gesekan pusaran air = 18.18 * 0.5 * 0.5 = 4.545 Newton
  - **Total Hambatan Air Maju = 2.015 + 4.545 = 6.56 Newton**
  - Motor harus memberikan gaya dorong minimal **6.56 Newton** hanya untuk mempertahankan kecepatan 0.5 m/s tersebut.

---

## 4. 🤿 Perhitungan Gaya Apung & Kenapa Kapal Anti-Tenggelam (Fail-Safe)

Robot AUV ini dirancang dengan prinsip **Gaya Apung Positif Alami (Positive Buoyancy)** sehingga mustahil tenggelam ke dasar kolam saat terjadi keadaan darurat.

### Perhitungan Langkah demi Langkah:

#### 1. Menghitung Berat Total Kapal di Udara (W):
- Massa kapal (ditimbang saat kering): m = 11.5 kg
- Percepatan gravitasi bumi: g = 9.807 meter/detik kuadrat
- **Gaya Berat (W) = 11.5 kg * 9.807 = 112.78 Newton (arah ke bawah)**

#### 2. Menghitung Gaya Angkat Air Archimedes (B):
- Densitas air kolam: rho = 998.2 kg/m³
- Volume total bodi dan spons busa apung: V = 0.01225 m³ (setara 12.25 liter)
- **Gaya Apung (B) = 998.2 * 9.807 * 0.01225 = 119.91 Newton (arah ke atas)**

#### 3. Menghitung Gaya Bersih (Sisa Gaya Apung):
- **Gaya Bersih ke Atas = Gaya Apung (B) - Gaya Berat (W)**
- **Gaya Bersih ke Atas = 119.91 Newton - 112.78 Newton = +7.13 Newton**
- Angka +7.13 Newton ini setara dengan daya angkat sebesar **+728 gram**.

### 🛡️ Fitur Keselamatan (Fail-Safe):
- Karena Gaya Apung lebih besar daripada Berat Kapal (selisih +7.13 Newton), maka jika baterai habis, kabel komunikasi putus, atau program error, kedua motor vertikal akan mati.
- Akibatnya, **kapal akan secara otomatis melayang naik sendiri ke permukaan air** secara pasif dan aman tanpa memerlukan daya listrik sama sekali.

---

## 5. 🌀 Perhitungan 6 Motor Thruster: Cara Kapal Maju, Geser & Muter

Kapal digerakkan oleh **6 unit motor BlueRobotics T200 Brushless Thruster**:
- **4 Motor Sudut (T1, T2, T3, T4)**: Dipasang mendatar di 4 sudut kapal dengan kemiringan sudut **45 derajat**.
- **2 Motor Vertikal (T5, T6)**: Dipasang tegak lurus di tengah depan dan tengah belakang kapal.

```
                  ▲ Moncong Depan (+X)
                  │
      T1 (45°)  ┌───┐  T2 (-45°)
        \       │   │       /
         \   ┌──┴───┴──┐   /
             │  [T5]   │     Lebar Kapal = 0.28 meter
             │ (Heave) │     Panjang Kapal = 0.54 meter
             │         │
             │  [T6]   │
         /   └──┬───┬──┘   \
        /       │   │       \
      T3 (135°) └───┘  T4 (-135°)
                  │
                  ▼ Ekor Belakang (-X)
```

---

### Cara Kerja Gerakan Motor:

1. **Maju Lurus**:
   - Motor T1, T2, T3, T4 menyala maju bersamaan.
   - Dorongan ke samping saling menghilangkan karena sudutnya 45 derajat berlawanan, menyisakan dorongan murni lurus ke depan.
2. **Geser Samping Kanan (Sway)**:
   - Motor T2 dan T3 dorong maju, Motor T1 dan T4 dorong mundur.
   - Hasilnya kapal bergeser murni ke kanan tanpa memutar badan.
3. **Putar Haluan di Tempat (Yaw)**:
   - Motor sisi kiri (T1, T3) dorong maju, Motor sisi kanan (T2, T4) dorong mundur.
   - Kapal berputar di tempat seperti tank baja.
4. **Menyelam ke Bawah**:
   - Motor vertikal T5 dan T6 menyala mendorong air ke atas, menekan kapal turun ke dalam air melawan gaya apung alami.

---

### Rumus Perhitungan Tenaga Tiap Motor (Matriks Alokasi Thruster):

Jika komputer pengendali meminta gaya maju **Fx**, gaya geser **Fy**, gaya selam **Fz**, dan momen putar **Mz**, maka tenaga yang dikirim ke masing-masing motor adalah:

- **Tenaga Motor T1 = (0.3536 * Fx) - (0.3536 * Fy) + (1.4141 * Mz)**
- **Tenaga Motor T2 = (0.3536 * Fx) + (0.3536 * Fy) - (1.4141 * Mz)**
- **Tenaga Motor T3 = (0.3536 * Fx) + (0.3536 * Fy) + (1.4141 * Mz)**
- **Tenaga Motor T4 = (0.3536 * Fx) - (0.3536 * Fy) - (1.4141 * Mz)**
- **Tenaga Motor T5 = (-0.5 * Fz) + (2.7778 * My)**
- **Tenaga Motor T6 = (-0.5 * Fz) - (2.7778 * My)**

*(Semua motor dibatasi maksimal tenaga dorong +50 Newton maju dan -40.2 Newton mundur pada tegangan baterai 16 Volt).*

---

## 6. 🛰️ Fusi Sensor Cerdas EKF 15-State

Di bawah air tidak ada sinyal GPS. Oleh karena itu, kapal harus menggabungkan beberapa sensor:
1. **Sensor IMU (100 Hz)**: Membaca percepatan dan laju putar secara sangat cepat, namun memiliki kelemahan mudah mengalami akumulasi error (drifting).
2. **Sensor Barometer Kedalaman (20 Hz)**: Mengukur tekanan air kolam untuk mengetahui kedalaman kapal secara akurat.
3. **Sensor DVL (10 Hz)**: Mengukur kecepatan kapal terhadap dasar kolam menggunakan pantulan gelombang suara akustik.

### Cara Kerja EKF (Extended Kalman Filter):
EKF bertindak seperti sistem cerdas yang menggabungkan 15 variabel keadaan kapal:
- 3 Posisi (X, Y, Z di kolam)
- 3 Kecepatan Linier (Maju u, Geser v, Selam w)
- 3 Sudut Orientasi (Roll, Pitch, Yaw)
- 3 Nilai Koreksi Bias Error Akselerometer
- 3 Nilai Koreksi Bias Error Giroskop

**Proses EKF**: Setiap milidetik EKF menebak posisi kapal menggunakan model fisika, lalu saat sensor kedalaman dan DVL mengirimkan data baru, EKF mengoreksi tebakan tersebut dan membuang gangguan noise sehingga estimasi posisi kapal tetap akurat dan tidak melenceng.

---

## 7. 🧠 Adaptasi Otomatis RLS saat Beban / Arus Air Berubah

### Mengapa butuh Algoritma RLS (Recursive Least Squares)?
Jika di kemudian hari kapal dipasangi kamera tambahan atau payload sensor baru, berat kapal akan bertambah dan hambatan airnya berubah.

Alih-alih harus menghitung ulang rumus secara manual, algoritma **Online RLS** di Digital Twin akan:
1. Membandingkan tenaga motor yang diberikan dengan kecepatan kapal yang dihasilkan.
2. Menghitung perubahan parameter hambatan air secara otomatis dalam waktu **1 detik**.
3. Menyesuaikan model Digital Twin secara langsung saat robot sedang beroperasi di kolam.

---

## 8. 🎯 Pengendali PID Kedalaman & Posisi

Pengendali PID mengatur tenaga motor agar kapal mencapai posisi atau kedalaman yang diperintahkan:
- **P (Proportional)**: Memberikan tenaga motor sebanding dengan jarak ke target. Semakin jauh dari target, motor mendorong semakin kuat.
- **I (Integral)**: Mengumpulkan error masa lalu untuk menghilangkan sisa penyimpangan kecil.
- **D (Derivative)**: Berfungsi sebagai rem halus saat kapal sudah mendekati target agar tidak kebablasan.

### Trik Khusus Pengendali Kedalaman (Feedforward Compensation):
Karena kapal memiliki gaya apung alami sebesar **+7.14 Newton** ke atas, maka pengendali kedalaman langsung memberikan tenaga dasar awal sebesar **-7.14 Newton (menekan ke bawah)**. Hasilnya, motor vertikal T5 dan T6 langsung mengunci kapal di kedalaman target (misalnya kedalaman 0.8 meter) secara stabil dan tenang tanpa goyang naik-turun.

---

## 9. 🔥 Bocoran Pertanyaan Dosen Penguji Sidang & Cara Menjawabnya

Gunakan panduan jawaban di bawah ini saat ditanya oleh dosen penguji:

---

#### ❓ Pertanyaan 1: *"Apa bedanya Digital Twin buatanmu dengan simulasi 3D biasa di Gazebo atau MATLAB?"*
> **Jawaban Tegas & Benar**:
> *"Perbedaan utamanya terletak pada **koneksi dua arah (bi-directional synchronization) dan adaptasi online**, Pak/Bu. Simulasi biasa hanya menjalankan matematika statis tanpa tahu kondisi robot nyata. Sedangkan Digital Twin kami terhubung langsung ke hardware Jetson Nano robot asli via ROS2 WebSocket. Jika robot asli di kolam terdorong arus air, Digital Twin langsung menyinkronkan posisinya secara real-time, mengoreksi error sensor dengan EKF, dan memperbarui parameter hambatan air menggunakan algoritma adaptif RLS."*

---

#### ❓ Pertanyaan 2: *"Kenapa 4 motor horizontal dipasang miring 45 derajat di 4 sudut, kenapa tidak lurus saja?"*
> **Jawaban Tegas & Benar**:
> *"Konfigurasi 45 derajat (vektorisasi TAM) memberikan kemampuan gerak **Holonomic / Omnidirectional** pada bidang datar. Artinya kapal dapat bergerak maju-mundur (Surge), bergeser murni ke samping kanan-kiri (Sway), dan berputar haluan (Yaw) secara bersamaan hanya dengan kombinasi 4 motor tersebut tanpa membutuhkan sirip kemudi mekanik."*

---

#### ❓ Pertanyaan 3: *"Bagaimana kamu membuktikan kapal tidak akan tenggelam dan hilang di dasar kolam jika baterai habis?"*
> **Jawaban Tegas & Benar**:
> *"Kapal dirancang dengan prinsip **Positive Buoyancy Fail-Safe**. Berdasarkan perhitungan Archimedes, gaya apung air ke atas adalah **119.91 Newton**, sedangkan berat kapal ke bawah adalah **112.78 Newton**. Terdapat selisih gaya angkat bersih ke atas sebesar **+7.14 Newton (setara +728 gram)**. Jadi jika baterai habis dan motor mati, kapal akan secara alami dan otomatis mengapung naik sendiri ke permukaan air tanpa perlu listrik."*

---

#### ❓ Pertanyaan 4: *"Apa itu Added Mass dan kenapa nilainya berbeda saat kapal maju dan saat kapal geser samping?"*
> **Jawaban Tegas & Benar**:
> *"Added Mass adalah massa fluida air di sekitar bodi yang ikut terseret saat kapal berakselerasi. Saat maju lurus, moncong kapal ramping sehingga air yang terdorong hanya **5.5 kg** (total beban 17 kg). Namun saat geser samping, penampang lambung kapal jauh lebih lebar sehingga air yang terseret mencapai **12.7 kg** (total beban 24.2 kg). Oleh karena itu tenaga dorong untuk geser samping harus lebih besar."*

---

## 10. 📋 Rangkuman Lengkap Data & Parameter Robot

| Data Parameter Robot | Angka Nyata | Satuan | Keterangan Praktis |
| :--- | :--- | :--- | :--- |
| **Massa Kering Kapal** | 11.5 | kilogram (kg) | Berat ditimbang di darat |
| **Ukuran Bodi (P x L x T)** | 0.54 x 0.28 x 0.24 | meter (m) | Dimensi rangka terluar |
| **Volume Total Kapal** | 0.01225 | meter kubik (12.25 Liter) | Volume benaman air |
| **Gaya Berat ke Bawah (W)** | 112.78 | Newton (N) | Akibat gravitasi bumi |
| **Gaya Apung ke Atas (B)** | 119.91 | Newton (N) | Hukum Archimedes air kolam |
| **Gaya Apung Bersih (+)** | +7.14 | Newton (setara +728 gram) | Kelebihan gaya apung pasif |
| **Massa Beban Maju Total** | 17.0 | kilogram (kg) | 11.5 kg bodi + 5.5 kg air |
| **Massa Beban Geser Total** | 24.2 | kilogram (kg) | 11.5 kg bodi + 12.7 kg air |
| **Massa Beban Selam Total** | 26.1 | kilogram (kg) | 11.5 kg bodi + 14.6 kg air |
| **Tipe Motor Thruster** | BlueRobotics T200 | 6 Unit | 4 horizontal 45°, 2 vertikal |
| **Dorongan Maksimal Motor** | 50.0 | Newton (5.1 kgf per motor) | Pada tegangan 16 Volt |
| **Baterai Robot** | LiPo 4-Cell (16.0V) | 10.000 mAh | Daya operasional kapal |

---

## 11. 🚀 Cara Cepat Menjalankan Program (Demo Sidang)

1. **Buka Aplikasi Dashboard 3D Digital Twin**:
   - Klik ganda file: **`Launch_Desktop_App.bat`** (Membuka antarmuka 3D visualisasi real-time).
2. **Jalankan Simulasi Backend ROS 2**:
   - Klik ganda file: **`simulator/start_simulation.bat`**.
3. **Jalankan Visualisasi Gazebo WSL2 (Opsional)**:
   - Klik ganda file: **`Launch_Gazebo_WSL.bat`**.
