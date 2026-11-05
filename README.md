# Lab ROS 2: kontrol dan pelacakan jalur

Repo ini berisi dua lab hands-on. Lab 1 membahas PID dan anti-windup lewat simulasi kendali kecepatan. Lab 2 membahas Pure Pursuit dan pengaruh jarak lookahead terhadap perilaku robot di tikungan. Kedua lab memakai node ROS 2 yang sudah tersedia; tugas peserta adalah menjalankan, mengubah parameter, lalu mencatat hasilnya. Setelah kedua lab selesai, ada lab lanjutan berdurasi tiga hari berisi permasalahan hands-on dan laporan riset, tersimpan di `LAB_LANJUTAN.md`.

## Prasyarat

- Ubuntu 24.04
- ROS 2 Jazzy sudah terpasang
- git

Cek dulu di terminal:

```bash
echo $ROS_DISTRO
git --version
```

`echo $ROS_DISTRO` harus mencetak `jazzy`. Kalau `git` tidak ditemukan, pasang dulu:

```bash
sudo apt install -y git
```

## Inisialisasi

```bash
source /opt/ros/jazzy/setup.bash
git clone git@github.com:rasyaarya/intro-to-control-CN16.git
cd intro-to-control-CN16
colcon build
source install/setup.bash
```

Baris pertama memuat ROS 2 dan aman dijalankan walau lingkungan ROS sudah termuat. `colcon build` berhenti dengan `Summary: 2 packages finished`; kalau muncul baris merah atau `colcon: command not found`, lihat bagian "Kalau ada masalah".

Kalau mesin belum punya kunci SSH GitHub, pakai URL HTTPS:

```bash
git clone https://github.com/rasyaarya/intro-to-control-CN16.git
```

`source install/setup.bash` hanya berlaku untuk terminal yang sedang dipakai. Setiap terminal baru perlu `cd` ke folder repo dan menjalankan `source install/setup.bash` lagi.

Perintah selanjutnya mengasumsikan repo ada di folder home (`~/intro-to-control-CN16`). Sesuaikan `cd` kalau disimpan di tempat lain.

## Lab 1: PID dan anti-windup

Pertanyaan: mengapa robot meluncur melewati target setelah tertahan tanjakan?

Node `pid_node` menjalankan dua simulasi sekaligus, satu tanpa anti-windup dan satu dengan anti-windup, lalu mencetak tabel kecepatan puncak dan kecepatan akhir. Parameter PID dibaca dari `src/pid_lab/config/params.yaml`.

### Task 1: jalankan baseline

Langkah:

1. Buka dua terminal.
2. Di terminal 1, jalankan dulu supaya sempat menangkap data:

   ```bash
   cd ~/intro-to-control-CN16
   source install/setup.bash
   ros2 topic echo /robot/speed std_msgs/msg/Float64
   ```

3. Di terminal 2:

   ```bash
   cd ~/intro-to-control-CN16
   source install/setup.bash
   ros2 run pid_lab pid_node --ros-args --params-file src/pid_lab/config/params.yaml
   ```

Amati angka yang mengalir di terminal 1, lalu tabel yang dicetak terminal 2 setelah simulasi selesai, sekitar 10 detik. Mode dengan anti-windup diterbitkan di `/robot/speed_aw` kalau ingin dibandingkan.

Catat untuk `jawaban/lab1.md`:

- kecepatan puncak mode tanpa anti-windup
- kecepatan puncak mode dengan anti-windup
- tangkapan layar tabel di terminal 2

Setelah tabel tercetak, hentikan `ros2 topic echo` di terminal 1 dengan Ctrl+C.

### Task 2: matikan suku integral

Langkah:

1. Buka file parameter di terminal 2:

   ```bash
   nano src/pid_lab/config/params.yaml
   ```

2. Ubah baris `ki: 6.0` menjadi `ki: 0.0`. Simpan dengan Ctrl+O lalu Enter, keluar dengan Ctrl+X.
3. Jalankan ulang node (tidak perlu build ulang):

   ```bash
   ros2 run pid_lab pid_node --ros-args --params-file src/pid_lab/config/params.yaml
   ```

Kalau terminal 2 sudah ditutup, jalankan dulu `cd ~/intro-to-control-CN16` dan `source install/setup.bash` di terminal baru sebelum perintah di atas.

Catat: kecepatan akhir kedua baris, selisihnya dengan baris dengan anti-windup Task 1, lalu satu kalimat tentang fungsi suku integral. Kedua baris akan sama persis. Itu wajar, anti-windup hanya berpengaruh saat integrator aktif.

### Task 3: perbesar suku integral

Langkah:

1. Buka lagi file parameter:

   ```bash
   nano src/pid_lab/config/params.yaml
   ```

2. Ubah `ki: 6.0` menjadi `ki: 12.0`, simpan, lalu jalankan ulang node:

   ```bash
   ros2 run pid_lab pid_node --ros-args --params-file src/pid_lab/config/params.yaml
   ```

Catat: puncak tanpa anti-windup, puncak dengan anti-windup, selisihnya dengan Task 1, dan kesimpulan apakah overshoot naik.

### Task 4: rangkum

Isi semua bagian `jawaban/lab1.md`, termasuk paragraf jawaban atas pertanyaan lab. Semua klaim harus memakai angka dari tabel.

## Lab 2: Pure Pursuit dan jarak lookahead

Pertanyaan: bagaimana jarak lookahead (`Ld`) mengubah perilaku robot di tikungan?

Node `pursuit_node` menjalankan satu putaran lintasan kotak 5 m kali 5 m dengan kendaraan model sepeda (roda depan bisa belok, lebar sumbu roda 0,5 m). Setiap selesai, node mencetak metrik galat lintasan (CTE), jumlah osilasi kemudi, dan waktu tempuh, lalu menulis grafik jejak ke file `jejak_pursuit_*.svg` di root repo. Parameter dibaca dari `src/pursuit_lab/config/params.yaml`, nilai `Ld` bisa ditimpa lewat baris perintah tanpa mengubah file.

Setelah selesai mencetak, node sengaja ditahan supaya topiknya tetap bisa dibuka di RViz2. Tekan Ctrl+C untuk mengakhiri sebelum menjalankan perintah berikutnya.

### Task 1: sweep empat nilai Ld

Buka satu terminal:

```bash
cd ~/intro-to-control-CN16
source install/setup.bash
```

Jalankan keempat perintah berikut satu per satu. Antar perintah, akhiri node terlebih dahulu dengan Ctrl+C:

```bash
ros2 run pursuit_lab pursuit_node --ros-args --params-file src/pursuit_lab/config/params.yaml -p ld:=0.3
```

```bash
ros2 run pursuit_lab pursuit_node --ros-args --params-file src/pursuit_lab/config/params.yaml -p ld:=0.8
```

```bash
ros2 run pursuit_lab pursuit_node --ros-args --params-file src/pursuit_lab/config/params.yaml -p ld:=1.5
```

```bash
ros2 run pursuit_lab pursuit_node --ros-args --params-file src/pursuit_lab/config/params.yaml -p ld:=3.0
```

Satu putaran memakan waktu sekitar 20 detik. Setiap run menghasilkan satu file grafik `jejak_pursuit_ld<nilai>_v<kecepatan>.svg`.

Catat untuk `jawaban/lab2.md`, satu baris per nilai `Ld`:

- CTE maksimum dan CTE rata-rata
- osilasi kemudi
- waktu tempuh

### Task 2: pilih Ld terbaik

Dari tabel Task 1, tentukan nilai `Ld` terbaik untuk lintasan ini. Tulis pilihannya di `jawaban/lab2.md` beserta alasan yang memakai angka, bukan perasaan. Perhatikan juga apa yang terjadi pada dua ujung: `Ld` terlalu kecil dan `Ld` terlalu besar sama-sama jelek, tapi jeleknya berbeda.

### Task 3: Ld adaptif terhadap kecepatan

Mode adaptif menghitung `Ld = ld_k * speed + ld_min` sehingga jarak lookahead ikut membesar kalau robot melaju lebih cepat.

Langkah:

1. Buka file parameter:

   ```bash
   nano src/pursuit_lab/config/params.yaml
   ```

2. Ubah baris `ld_adaptive: false` menjadi `ld_adaptive: true`. Simpan dengan Ctrl+O lalu Enter, keluar dengan Ctrl+X.
3. Jalankan pada dua kecepatan (akhiri tiap run dengan Ctrl+C):

   ```bash
   ros2 run pursuit_lab pursuit_node --ros-args --params-file src/pursuit_lab/config/params.yaml -p speed:=0.5
   ```

   ```bash
   ros2 run pursuit_lab pursuit_node --ros-args --params-file src/pursuit_lab/config/params.yaml -p speed:=2.0
   ```

Baris log di awal mencetak `Ld` efektif yang dipakai. Catat di `jawaban/lab2.md`: kecepatan, `Ld` efektif, CTE maksimum, CTE rata-rata, dan osilasi kemudi untuk kedua kecepatan.

Jawab dengan satu paragraf: kenapa `Ld` dibuat fungsi kecepatan, bukan nilai tetap? Pakai angka dari kedua run.

### Task 4 (opsional): lihat di RViz2

Node dan RViz2 bisa dijalankan berurutan atau berbareangan; yang penting keduanya berjalan bersamaan. Buka terminal tambahan:

```bash
cd ~/intro-to-control-CN16
source install/setup.bash
rviz2 -d src/pursuit_lab/config/pursuit.rviz
```

Lalu jalankan ulang salah satu perintah node dari Task 1 atau Task 3 di terminal lain. Di RViz2 terlihat kotak hijau (lintasan target) dan jejak merah robot di atasnya. Tutup RViz2 dengan Ctrl+C.

### Task 5: rangkum

Isi semua bagian `jawaban/lab2.md`, termasuk jawaban dua paragraf atas pertanyaan lab. Sertakan file grafik `jejak_pursuit_*.svg` atau tangkapan layarnya.

## Format pengumpulan

- `jawaban/lab1.md`: nama, tabel hasil tiap task, satu tangkapan layar terminal, jawaban singkat.
- `jawaban/lab2.md`: dua tabel hasil, pilihan `Ld` terbaik beserta alasan berbasis angka, dua paragraf jawaban, grafik atau tangkapan layar.

Jawaban tanpa angka dianggap belum selesai.

## Kalau ada masalah

| Gejala | Penyebab dan tindakan |
| --- | --- |
| `ros2: command not found` | Lingkungan ROS belum dimuat. Jalankan `source /opt/ros/jazzy/setup.bash` |
| `colcon: command not found` | Colcon belum terpasang. Jalankan `sudo apt install -y python3-colcon-common-extensions`, lalu ulangi blok Inisialisasi |
| `Package 'pid_lab' not found` atau `Package 'pursuit_lab' not found` | `source install/setup.bash` belum dijalankan di terminal itu, atau build dijalankan dari folder selain root repo |
| Angka tidak berubah setelah edit YAML | File yang diedit salah, atau perintah dijalankan dari luar folder repo. Path parameter harus `src/pid_lab/config/params.yaml` (Lab 1) atau `src/pursuit_lab/config/params.yaml` (Lab 2) |
| Terminal 1 Lab 1 tidak menampilkan apa pun | Perintah echo harus memuat tipe topik: `ros2 topic echo /robot/speed std_msgs/msg/Float64`. Data muncul setelah node dijalankan di terminal 2 |
| Node Lab 2 berhenti dengan "robot menyimpang lebih dari 4 m" | Kombinasi `Ld` dan `speed` terlalu ekstrem sehingga robot gagal mengikuti lintasan. Jalankan ulang dengan nilai yang lebih wajar |
| RViz2 terbuka tetapi tidak ada garis | Node belum berjalan, atau Status pada panel Displays merah. Jalankan node dulu, lalu periksa jalur topik `/pursuit/target_path` dan `/pursuit/trace` |

## Referensi

- Dokumentasi ROS 2 Jazzy: https://docs.ros.org/en/jazzy/
- Dokumentasi RViz2: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz2/Getting-Started.html
- Lab lanjutan, tenggat tiga hari: `LAB_LANJUTAN.md`
- PythonRobotics: https://github.com/AtsushiSakai/PythonRobotics
