# Lab lanjutan: kendali dan navigasi

Tenggat pengerjaan tiga hari sejak sesi. Lab lanjutan menghasilkan dua produk yang dikumpulkan bersamaan:

1. laporan hands-on berisi solusi tiga permasalahan pengerjaan di repo ini,
2. laporan riset satu topik kendali dari tinjauan pustaka.

Prasyarat: Lab 1 dan Lab 2 di `README.md` sudah selesai, termasuk `jawaban/lab1.md` dan `jawaban/lab2.md`. Angka dari kedua lab itu sah dipakai sebagai pembanding pada laporan hands-on.

## Ketentuan umum

- Hands-on dikerjakan di repo `intro-to-control-CN16` pada mesin masing-masing.
- Saran pembagian: hari pertama Masalah 1, hari kedua Masalah 2 dan 3, hari ketiga laporan riset.
- Kode dan laporan dikerjakan sendiri. Sumber luar, potongan kode, dan referensi yang dipakai wajib disebutkan.
- Setiap pernyataan hasil wajib menyertakan perintah, parameter, dan angka. Laporan tanpa angka tidak diterima.
- Hands-on yang sudah selesai tidak boleh merusak jalur Lab 1 dan Lab 2. Kalau ada kode lama yang perlu diubah, salin dulu paketnya.

## Bagian 1: permasalahan hands-on

### Masalah 1: Stanley controller dan gangguan posisi

**Pertanyaan:** presisi Stanley sepadan dengan beban pada aktuator?

Tulis node ROS 2 baru dari nol, nama paket `stanley_lab`. Pakai isi `src/pursuit_lab` sebagai contoh struktur: parameter dibaca dari file YAML, lintasan kotak 5 m x 5 m dipakai ulang, metrik yang dicetak sama seperti Lab 2, node ditahan setelah mencetak supaya RViz2 masih bisa dibuka.

Hukum kendali Stanley:

```
delta = theta_e + atan2(k * e, v)
```

dengan `e` galat lintasan lateral (m), `theta_e` selisih arah robot terhadap arah lintasan (rad), `v` kecepatan maju (m/s), `k` kecepatan pengendalian tanpa satuan. Hasilnya dijaga `|delta| <= 0,6` rad seperti `pursuit_lab`.

Dua gangguan ditambahkan di dalam node sebelum perhitungan kendali:

- posisi awal robot digeser 0,5 m lateral dari lintasan dan arah awalnya 15 derajat menyimpang;
- pengukuran posisi diberi derau Gaussian deviasi 0,02 m dengan benih acak tetap supaya hasil bisa diulang.

Uji empat konfigurasi, masing-masing pada kondisi bersih dan kondisi gangguan:

| Konfigurasi | Nilai |
| --- | --- |
| Pure Pursuit | `Ld = 0,8` m |
| Stanley | `k = 1`, `k = 2`, `k = 4` |

Untuk menambahkan gangguan ke Pure Pursuit, salin dulu `pursuit_lab` ke paket baru agar Lab 2 tetap utuh. Selain metrik Lab 2, tambahkan pencetakan total variasi kemudi (jumlah selisih sudut kemudi antar langkah) pada kedua node, karena metrik inilah yang dipakai menilai beban aktuator.

Kriteria diterima:

- seluruh run berjalan tanpa galat dan menghasilkan tabel delapan baris: CTE maksimum, CTE rata-rata, total variasi kemudi, sudut kemudi maksimum, waktu tempuh;
- bila salah satu nilai `k` membuat robot gagal mengikuti lintasan (node membatalkan putaran), laporkan nilai itu sebagai kondisi tidak stabil dan jelaskan penyebabnya secara teori;
- satu paragraf kesimpulan yang menjawab pertanyaan memakai angka dari tabel, termasuk nilai `k` dengan kompromi terbaik antara presisi dan beban kemudi.

### Masalah 2: perencanaan Hybrid A*

**Pertanyaan:** mengapa A* biasa menghasilkan jalur yang tidak bisa diikuti mobil?

Tulis dua perencana dari nol di Python tanpa ROS, hanya memakai `numpy` dan `matplotlib`: A* pada grid okupansi dua dimensi, dan Hybrid A* dengan motion primitives serta model kendaraan sumbu roda 0,5 m seperti Lab 2.

Peta uji dibakukan: petak 30 m x 30 m, dinding keliling, dua blok rintangan 6 m x 6 m berpusat di (10, 10) dan (20, 20), titik awal (2, 2) dengan arah 0 derajat, tujuan (28, 28) dengan arah 90 derajat.

Ukur pada tiga nilai radius belok minimum: 1 m, 2 m, dan 4 m. Yang dicatat: panjang jalur, waktu komputasi rata-rata dari sepuluh kali jalur, jumlah simpul yang diekspansi, dan sudut belok terbesar pada jalur A* 2D dibanding batas radius.

Kriteria diterima:

- dua skrip berjalan masing-masing dengan satu perintah dan mengeluarkan plot peta berisi rintangan, jalur A*, serta jalur Hybrid A*;
- jalur Hybrid A* kinematis valid pada radius 1 m dan 2 m (kelengkungan maksimum jalur tidak melampaui `1/R`); kalau radius 4 m tidak menemukan jalur, itu temuan yang dilaporkan beserta alasannya;
- tabel pengukuran lengkap dan satu paragraf yang menjawab pertanyaan memakai angka tabel.

### Masalah 3: integrasi perencanaan dan pelacakan

**Pertanyaan:** seberapa tahan tiap pengendali terhadap jalur berbelokan tajam?

Ambil jalur hasil Masalah 2 dengan radius 2 m, ubah ke format berkas titik yang bisa dimuat kedua node (YAML atau CSV, formatnya Anda yang tentukan dan dokumentasikan). Muat berkas itu di `stanley_lab` dan di paket Pure Pursuit salinan dari Masalah 1, lalu jalankan masing-masing pada kecepatan 1,0 m/s dan 2,0 m/s. Kedua node wajib memakai berkas jalur yang identik.

Kriteria diterima:

- tabel empat baris berisi CTE maksimum, CTE rata-rata, total variasi kemudi, dan waktu tempuh;
- penanda pada plot di titik jalur yang kelengkungannya melebihi batas kemampuan pengendali, beserta komentar apa yang terjadi di sana;
- satu paragraf: pengendali mana yang layak dipakai untuk jalur tajam berkecepatan tinggi, dengan angka dari keempat run.

### Masalah 4 (opsional, tambahan nilai): ganti controller di Nav2

Butuh lingkungan Nav2, TurtleBot3, dan Gazebo. Berkas pendukung (`nav2/params_dwb.yaml`, `nav2/params_rpp.yaml`, `nav2/params_mppi.yaml`) disiapkan instruktur sebelum sesi; sampai berkas itu tersedia, kerjakan Masalah 1 sampai 3 lebih dulu lalu hubungi instruktur.

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=nav2/params_dwb.yaml
```

Ulangi dengan `params_rpp.yaml` dan `params_mppi.yaml`, kirim goal yang sama lewat RViz pada tiap percobaan.

| Controller | Waktu tempuh | Jarak terdekat ke dinding | Berhenti atau macet? |
| --- | --- | --- | --- |
| DWB | | | |
| Regulated Pure Pursuit | | | |
| MPPI | | | |

> Catatan instruktur: nama paket dan plugin berubah antarversi Nav2. Sebelum sesi, cek daftar plugin resmi di dokumentasi Nav2 untuk versi ROS 2 yang dipakai dan sesuaikan file YAML.

**Tugas:** tentukan controller mana yang paling dekat dengan rintangan, ubah satu parameter kecepatan maksimum, dan catat dampaknya.

### Masalah 5 (opsional, tambahan nilai): Behavior Tree dan recovery

Butuh berkas `bt_recovery.xml` dari instruktur dan lingkungan Masalah 4.

1. Jalankan simulasi seperti Masalah 4.
2. Letakkan kotak di tengah koridor lewat Gazebo saat robot berjalan.
3. Amati urutan recovery di terminal Nav2: clear costmap, spin, wait, backup.
4. Salin `bt_recovery.xml`, ubah urutan atau hapus satu recovery, jalankan ulang.

**Tugas:** gambarkan Behavior Tree bawaan dan versi modifikasi, sebutkan skenario yang gagal setelah modifikasi. Opsional: visualisasikan dengan Groot.

## Bagian 2: laporan riset

Satu laporan tinjauan pustaka tentang teori atau penerapan kendali. Topik bebas selama masih seputar kendali, contohnya: perbandingan metode anti-windup, PID terhadap MPC pada aktuator berbatas, pelacakan jalur untuk kendaraan otonom, kendali kecepatan adaptif, estimasi slip roda, LQR untuk kendaraan. Pilih satu subtopik yang sempit, jangan merangkum seluruh bidang.

Struktur laporan, urutan wajib:

1. Abstrak, maksimal 200 kata.
2. Pendahuluan: masalah, alasan pemilihan topik, ruang lingkup.
3. Tinjauan pustaka: landasan teori dan minimal enam sumber, sekurang-kurangnya tiga artikel ilmiah dari jurnal atau prosiding, sisanya buku teks, dokumentasi resmi, atau repositori kode terbuka.
4. Pembahasan: bandingkan sedikitnya tiga pendekatan atau hasil dari berbagai sumber, sertakan persamaan yang dipakai.
5. Relevansi dengan praktik: satu bagian wajib yang menghubungkan isi tinjauan dengan angka dari Lab 1, Lab 2, atau hasil Masalah 1 sampai 3.
6. Kesimpulan yang menjawab pertanyaan yang dirumuskan di pendahuluan.
7. Referensi bernomor konsisten, gaya IEEE.

Panjang 2500 sampai 3500 kata, bahasa Indonesia, istilah teknis boleh berbahasa Inggris. Setiap angka atau klaim yang berasal dari sumber lain wajib disitasi.

Kriteria penilaian: argumen disusun sendiri, bukan ringkasan tempelan dari beberapa artikel; bagian relevansi memakai angka nyata dari pengerjaan sendiri; referensi tertulis lengkap dan dapat ditelusuri.

## Pengumpulan

- `jawaban/lanjutan_hands_on.md` — laporan Masalah 1 sampai 3: perintah yang dijalankan, seluruh tabel, tangkapan layar atau grafik, dan analisis tiap masalah.
- Kode hands-on masuk repo: paket `src/stanley_lab/`, skrip perencanaan di folder `perencana/` yang Anda buat sendiri, beserta petunjuk menjalankannya yang ditulis di laporan.
- `jawaban/lanjutan_riset.pdf` — laporan riset sesuai struktur di atas.

Masalah 4 dan 5 hanya dikumpulkan kalau dikerjakan.

## Referensi

- Dokumentasi ROS 2 Jazzy: https://docs.ros.org/en/jazzy/
- Dokumentasi Nav2: https://docs.nav2.org
- PythonRobotics: https://github.com/AtsushiSakai/PythonRobotics
- Coulter, R. C. (1992). Implementation of the Pure Pursuit Path Tracking Algorithm. Carnegie Mellon University.
- Thrun, S. et al. (2006). Stanley: The Robot That Won the DARPA Grand Challenge. Journal of Artificial Intelligence Research.
