# Lab 1: PID dan anti-windup

Nama: Danar Abdi Manaf Hamid
NIM: 5025261164

## Task 1: baseline (kp=5, ki=6, kd=0)

| Mode | Puncak (m/s) | Akhir (m/s) |
| --- | --- | --- |
| tanpa anti-windup | 2.39| 1.05|
| dengan anti-windup | 1.06| 1.00|

Tangkapan layar terminal:
danarabdi@danarabdi-ASUS-TUF-Gaming-A16-FA607NUG-FA607NUG:~/intro-to-control-CN16$ source install/setup.bash
ros2 run pid_lab pid_node --ros-args --params-file src/pid_lab/config/params.yaml
[INFO] [1793885237.126326681] [pid_node]: parameter: kp=5.00 ki=6.00 kd=0.00

mode                    puncak (m/s)  akhir (m/s)
-------------------------------------------------
tanpa anti-windup               2.39         1.05
dengan anti-windup              1.06         1.00

## Task 2: suku integral dimatikan (ki=0)

Kecepatan akhir kedua baris: 0.83
Selisih dengan baris dengan anti-windup Task 1: 0.17
Fungsi suku integral: menjumlahkan error terus-menerus supaya kecepatan akhir bisa sampai ke target. Buktinya di Task 2: kalau ki=0, kecepatan akhir cuma 0,83 m/s, kurang 0,17 m/s dari Task 1 (1,00 m/s)

## Task 3: suku integral diperbesar (ki=12)

Puncak tanpa anti-windup: 2.59
Puncak dengan anti-windup: 1.09
Selisih dengan Task 1: akhir tanpa anti-windup : 0.05 akhir Dengan anti-windup : 0, 
Kesimpulan soal overshoot: ketika ki nya dinaikan puncak tanpa anti-windup semakin besar, yang menyebabkan overshoot yang semakin besar juga. Tetapi dengan anti-windup puncak kecepatan sangat dekat(selisih tidak jauh), artinya ki yang lebih besar memperburuk overshoot jika tidak menggunakan anti-windup. Jadi anti-windup menjaga overshoot tetap kecil walaupun ki dinaikan. 

## Paragraf
Jadi, kontrol proporsional (P) itu kerjanya gaya dorong robot sebanding dengan error (selisih antara target dan kecepatan sekarang). Makin besar error, makin besar dorongannya.
Masalahnya, kalau robot sudah dekat target, error-nya makin kecil, jadi dorongannya juga ikut mengecil. Padahal tanjakan terus menahan robot. Akhirnya ada titik di mana dorongan yang tersisa pas-pasan buat melawan tanjakan, dan robot berhenti naik kecepatannya di situ, sebelum sampai target. Itu yang terjadi di Task 2: robot mentok di 0,83 m/s, padahal targetnya 1,0 m/s. Sisa selisih 0,17 m/s itu tidak bisa ditutup, karena kalau error sudah kecil, dorongan P-nya juga kecil.
Suku integral yang menyelesaikan ini. Selama masih ada error sekecil apa pun, integral terus menambah dorongan sedikit demi sedikit, sampai error benar-benar hilang. Makanya di Task 1 (ki=6) kecepatan akhirnya sampai 1,00 m/s.
