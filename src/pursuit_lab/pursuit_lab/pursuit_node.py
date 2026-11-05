"""Pure Pursuit: robot mengikuti lintasan kotak dengan jarak lookahead Ld.

Parameter dibaca dari config/params.yaml. Node menjalankan satu putaran
lintasan, mencetak metrik galat lintasan, menerbitkan topik RViz, lalu
menulis grafik jejak ke file SVG.
"""

import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
from tf2_ros import StaticTransformBroadcaster

# lintasan: kotak 5 m x 5 m, mulai dari (1, 0) menghadap +x,
# simulasi berhenti setelah kembali ke titik awal
PATH_POINTS = [
    (1.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0), (1.0, 0.0)]
WHEELBASE = 0.5      # jarak antar sumbu roda, m
DELTA_MAX = 0.6      # batas sudut kemudi, rad
DT = 0.05            # langkah simulasi, s
STEER_TAU = 0.15     # konstanta waktu sistem kemudi, s
MAX_TIME = 60.0      # batas waktu simulasi, s
TIME_SCALE = 4.0     # simulasi diputar 4x lebih cepat dari waktu nyata
PUBLISH_EVERY = 2    # terbitkan tiap 2 langkah, 0,1 s simulasi
FRAME = 'map'

_SEG = [math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(PATH_POINTS, PATH_POINTS[1:])]
_CUM = [0.0]
for _s in _SEG:
    _CUM.append(_CUM[-1] + _s)
_TOTAL = _CUM[-1]


def _closest_point(x, y, start_i):
    """Jarak terpendek (x, y) ke jalur, dicari maju dari indeks start_i."""
    best_d = float('inf')
    best = (start_i, 0.0)
    for i in range(start_i, len(PATH_POINTS) - 1):
        ax, ay = PATH_POINTS[i]
        bx, by = PATH_POINTS[i + 1]
        vx, vy = bx - ax, by - ay
        vv = vx * vx + vy * vy
        t = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / vv))
        d = math.hypot(x - (ax + t * vx), y - (ay + t * vy))
        if d < best_d:
            best_d = d
            best = (i, t)
    return best_d, best[0], best[1]


def _lookahead_point(progress, ld):
    """Titik pada jalur yang berjarak ld dari posisi busur saat ini."""
    target_s = min(progress + ld, _TOTAL)
    tx, ty = PATH_POINTS[-1]
    for i in range(len(_SEG)):
        if _CUM[i + 1] >= target_s:
            frac = 0.0 if _SEG[i] == 0 else (target_s - _CUM[i]) / _SEG[i]
            tx = PATH_POINTS[i][0] + frac * (PATH_POINTS[i + 1][0] - PATH_POINTS[i][0])
            ty = PATH_POINTS[i][1] + frac * (PATH_POINTS[i + 1][1] - PATH_POINTS[i][1])
            break
    return tx, ty


def simulate(ld, speed, on_step=None):
    """Satu putaran lintasan.

    Mengembalikan (metrik, jejak, selesai). metrik None artinya robot
    menyimpang terlalu jauh dari lintasan.
    """
    x, y = PATH_POINTS[0]
    th = 0.0
    delta_act = 0.0
    prev_delta = 0.0
    idx = 0
    t = 0.0
    peak = 0.0
    cte_sum = 0.0
    n = 0
    osc = 0
    steps = 0
    trace = [(x, y)]

    while t < MAX_TIME:
        cte, idx, tt = _closest_point(x, y, idx)
        progress = _CUM[idx] + tt * _SEG[idx]
        if progress >= _TOTAL - 1e-6:
            break
        if (progress > 0.9 * _TOTAL and
                math.hypot(x - PATH_POINTS[-1][0], y - PATH_POINTS[-1][1]) < 0.15):
            break
        if cte > 4.0:
            return None, trace, False
        peak = max(peak, cte)
        cte_sum += cte
        n += 1

        tx, ty = _lookahead_point(progress, ld)
        alpha = math.atan2(ty - y, tx - x) - th
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        delta_cmd = math.atan2(2.0 * WHEELBASE * math.sin(alpha), max(ld, 0.2))
        delta_cmd = max(-DELTA_MAX, min(DELTA_MAX, delta_cmd))
        delta_act += (delta_cmd - delta_act) * min(1.0, DT / STEER_TAU)

        if abs(delta_act) > 1e-3 and prev_delta * delta_act < 0:
            osc += 1
        prev_delta = delta_act

        x += speed * math.cos(th) * DT
        y += speed * math.sin(th) * DT
        th += speed * math.tan(delta_act) / WHEELBASE * DT
        t += DT
        steps += 1

        if on_step is not None and steps % PUBLISH_EVERY == 0:
            on_step(t, cte)
            trace.append((x, y))
        time.sleep(DT / TIME_SCALE)

    done = t < MAX_TIME
    if n == 0:
        return None, trace, done
    metrics = {
        'cte_max': peak,
        'cte_mean': cte_sum / n,
        'osc': osc,
        'time': t,
    }
    return metrics, trace, done


def write_svg(path_file, trace, metrics, ld, speed):
    """Tulis grafik jejak lintasan ke file SVG (bisa dibuka browser)."""

    def px(x):
        return f'{x:.3f}'

    def py(y):
        return f'{-y:.3f}'

    target_pts = ' '.join(f'{px(x)},{py(y)}' for x, y in PATH_POINTS)
    trace_pts = ' '.join(f'{px(x)},{py(y)}' for x, y in trace)
    line1 = f'ld={ld:.2f} m   speed={speed:.1f} m/s'
    line2 = (f'CTE maks {metrics["cte_max"]:.2f} m | '
             f'rata-rata {metrics["cte_mean"]:.2f} m')
    line3 = (f'osilasi kemudi {metrics["osc"]} | '
             f'waktu tempuh {metrics["time"]:.1f} s')

    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="-0.5 -7.25 6 7.75">
  <rect x="-0.5" y="-7.25" width="6" height="7.75" fill="#ffffff"/>
  <text x="-0.4" y="-6.95" font-family="monospace" font-size="0.30" fill="#222222">{line1}</text>
  <text x="-0.4" y="-6.58" font-family="monospace" font-size="0.25" fill="#444444">{line2}</text>
  <text x="-0.4" y="-6.21" font-family="monospace" font-size="0.25" fill="#444444">{line3}</text>
  <line x1="-0.4" y1="-5.85" x2="0.1" y2="-5.85" stroke="#2e7d32" stroke-width="0.07"/>
  <text x="0.2" y="-5.77" font-family="monospace" font-size="0.25" fill="#333333">lintasan target</text>
  <line x1="2.7" y1="-5.85" x2="3.2" y2="-5.85" stroke="#c62828" stroke-width="0.07"/>
  <text x="3.3" y="-5.77" font-family="monospace" font-size="0.25" fill="#333333">jejak robot</text>
  <polyline points="{target_pts}" fill="none" stroke="#2e7d32" stroke-width="0.05"/>
  <polyline points="{trace_pts}" fill="none" stroke="#c62828" stroke-width="0.04"/>
  <circle cx="{px(PATH_POINTS[0][0])}" cy="{py(PATH_POINTS[0][1])}" r="0.12" fill="#1565c0"/>
</svg>
'''
    with open(path_file, 'w', encoding='ascii') as f:
        f.write(svg)


class PursuitNode(Node):
    def __init__(self):
        super().__init__('pursuit_node')
        self.declare_parameter('ld', 0.8)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('ld_adaptive', False)
        self.declare_parameter('ld_k', 1.0)
        self.declare_parameter('ld_min', 0.5)
        self.ld = self.get_parameter('ld').value
        self.speed = self.get_parameter('speed').value
        self.ld_adaptive = self.get_parameter('ld_adaptive').value
        self.ld_k = self.get_parameter('ld_k').value
        self.ld_min = self.get_parameter('ld_min').value

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_path = self.create_publisher(Path, '/pursuit/target_path', latched)
        self.pub_trace = self.create_publisher(Path, '/pursuit/trace', latched)
        self.pub_cte = self.create_publisher(Float64, '/pursuit/cte', 10)

        # transformasi statis map -> odom agar RViz tidak melaporkan
        # "No transform from [odom] to [map]" pada fixed frame
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = FRAME
        tf.child_frame_id = 'odom'
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf)

    def _path_msg(self, points):
        msg = Path()
        msg.header.frame_id = FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        for px_, py_ in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = px_
            pose.pose.position.y = py_
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def run(self):
        if self.ld_adaptive:
            ld = self.ld_k * self.speed + self.ld_min
            mode = f'adaptif {self.ld_k:.2f}*v + {self.ld_min:.2f}'
        else:
            ld = self.ld
            mode = 'tetap'
        self.get_logger().info(
            f'parameter: ld={ld:.2f} m ({mode}), speed={self.speed:.2f} m/s')
        self.pub_path.publish(self._path_msg(PATH_POINTS))

        def on_step(t, cte):
            self.pub_cte.publish(Float64(data=cte))

        metrics, trace, done = simulate(ld, self.speed, on_step)
        self.pub_trace.publish(self._path_msg(trace))

        if metrics is None:
            print()
            print('robot menyimpang lebih dari 4 m dari lintasan, '
                  'putaran dibatalkan.')
            print('periksa kembali parameter ld dan speed.')
            time.sleep(0.3)
            return

        print()
        print(f'{"metrik":24s}{"nilai":>10s}')
        print('-' * 34)
        print(f'{"CTE maksimum (m)":24s}{metrics["cte_max"]:10.2f}')
        print(f'{"CTE rata-rata (m)":24s}{metrics["cte_mean"]:10.2f}')
        print(f'{"osilasi kemudi":24s}{metrics["osc"]:10d}')
        print(f'{"waktu tempuh (s)":24s}{metrics["time"]:10.1f}')
        if not done:
            print()
            print('catatan: putaran belum selesai dalam batas waktu simulasi.')
        name = f'jejak_pursuit_ld{ld:.2f}_v{self.speed:.1f}.svg'
        write_svg(name, trace, metrics, ld, self.speed)
        print()
        print(f'grafik : {os.path.abspath(name)}')
        print('topik  : /pursuit/target_path, /pursuit/trace (untuk RViz2), '
              '/pursuit/cte')
        time.sleep(0.3)

        # topik latched tetap hidup sampai node ditutup dengan Ctrl+C,
        # sehingga RViz2 boleh dibuka sebelum maupun sesudah simulasi
        print()
        print('node tetap aktif agar topik bisa dibuka di RViz2. '
              'Tutup dengan Ctrl+C.')
        while rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.2)
            except KeyboardInterrupt:
                break
            except Exception:
                break


def main(args=None):
    rclpy.init(args=args)
    node = PursuitNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
