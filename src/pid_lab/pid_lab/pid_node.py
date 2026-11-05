import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

TARGET = 1.0       # kecepatan target, m/s
DT = 0.02          # langkah simulasi, s
SIM_TIME = 10.0    # durasi simulasi, s
HILL_START = 2.5   # tanjakan mulai, s
HILL_END = 5.5     # tanjakan selesai, s
HILL_LOAD = 3.0    # beban tanjakan, m/s^2
DRAG = 1.0         # hambatan linear, 1/s
U_MAX = 3.0        # batas aktuator, m/s^2
PUBLISH_EVERY = 5  # terbitkan tiap 5 langkah, 0,1 s


class PIDSim:
    """Satu mode simulasi: PID dengan atau tanpa anti-windup."""

    def __init__(self, kp, ki, kd, anti_windup):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.anti_windup = anti_windup
        self.v = 0.0
        self.integral = 0.0
        self.prev_error = TARGET - self.v
        self.peak = 0.0

    def step(self, t):
        error = TARGET - self.v
        derivative = (error - self.prev_error) / DT
        self.prev_error = error
        hill = HILL_LOAD if HILL_START <= t < HILL_END else 0.0

        raw = self.kp * error + self.ki * self.integral + self.kd * derivative
        saturated = max(-U_MAX, min(U_MAX, raw)) != raw
        if not (self.anti_windup and saturated and error * raw > 0):
            self.integral += error * DT

        raw = self.kp * error + self.ki * self.integral + self.kd * derivative
        u = max(-U_MAX, min(U_MAX, raw))
        self.v += (u - DRAG * self.v - hill) * DT
        self.peak = max(self.peak, self.v)
        return self.v


class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_node')
        self.declare_parameter('kp', 5.0)
        self.declare_parameter('ki', 6.0)
        self.declare_parameter('kd', 0.0)
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.pub = self.create_publisher(Float64, '/robot/speed', 10)
        self.pub_aw = self.create_publisher(Float64, '/robot/speed_aw', 10)

    def run(self):
        self.get_logger().info(
            f'parameter: kp={self.kp:.2f} ki={self.ki:.2f} kd={self.kd:.2f}')
        plain = PIDSim(self.kp, self.ki, self.kd, anti_windup=False)
        with_aw = PIDSim(self.kp, self.ki, self.kd, anti_windup=True)

        steps = int(SIM_TIME / DT)
        for i in range(steps):
            t = i * DT
            v_plain = plain.step(t)
            v_aw = with_aw.step(t)
            if i % PUBLISH_EVERY == 0:
                self.pub.publish(Float64(data=v_plain))
                self.pub_aw.publish(Float64(data=v_aw))
            time.sleep(DT)

        print()
        print(f'{"mode":22s}{"puncak (m/s)":>14s}{"akhir (m/s)":>13s}')
        print('-' * 49)
        print(f'{"tanpa anti-windup":22s}{plain.peak:14.2f}{plain.v:13.2f}')
        print(f'{"dengan anti-windup":22s}{with_aw.peak:14.2f}{with_aw.v:13.2f}')
        print()


def main(args=None):
    rclpy.init(args=args)
    node = PIDNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
