import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math, time, signal, statistics


Q1 = 0.4
Q2 = 0.75
Q3 = 0.28
Q4 = 0.22
Q5 = -0.13
Q6 = 0.10
Q7 = 20
Q8 = 5
Q9 = 4
Q10 = 0.18
Q11 = 0.25
Q12 = 0.033
Q13 = 0.083
Q14 = 10.5


class _Nx(Node):
    def __init__(self):
        super().__init__('nx_runtime')
        self.create_subscription(LaserScan, '/scan', self._h, 10)
        self._p0 = self.create_publisher(Twist,  '/cmd_vel',       10)
        self._p1 = self.create_publisher(String, '/folkrace_olek', 10)
        self._z0 = 0.0
        self._z1 = 0
        self._z2 = 0
        self._z3 = 0
        self.get_logger().info('nx_runtime active')

    def _emit(self, s: str):
        self._p1.publish(String(data=s))

    def _sm(self, buf, a0: float, a1: float) -> float:
        n = len(buf)
        if not n:
            return float('inf')
        _ki = lambda k: int((k + 180.0) / 360.0 * n) % n
        i0, i1 = _ki(a0), _ki(a1)
        sl = buf[i0:i1+1] if i0 <= i1 else list(buf[i0:]) + list(buf[:i1+1])
        v = sorted(r for r in sl if not math.isnan(r) and not math.isinf(r) and 0.12 <= r <= 8.0)
        if not v:
            return float('inf')
        return v[max(0, int(len(v) * 0.20) - 1)]

    def _xd(self, raw):
        n = len(raw)
        sr = math.radians(180.0 / n)
        out = list(raw)
        for i in range(n - 1):
            dl, dr = raw[i], raw[i+1]
            if abs(dl - dr) < Q11:
                continue
            if dl < dr:
                d = dl
                if d < 0.15:
                    continue
                w = int(math.asin(min(1.0, Q10/d)) / sr) + 1
                for k in range(i+1, min(i+1+w, n)):
                    if out[k] > d: out[k] = d
                if i > 0 and out[i-1] > d*1.1: out[i-1] = d*1.1
            else:
                d = dr
                if d < 0.15:
                    continue
                w = int(math.asin(min(1.0, Q10/d)) / sr) + 1
                for k in range(max(0, i+1-w), i+1):
                    if out[k] > d: out[k] = d
                if i+2 < n and out[i+2] > d*1.1: out[i+2] = d*1.1
        return out

    def _cl(self, u: float, w: float):
        lim = (abs(u) + abs(w)*Q13) / Q12
        if lim > Q14:
            s = Q14 / lim
            if abs(u) < 0.05:
                w *= s
            else:
                u *= s
                w *= s
        return u, w

    def _h(self, msg: LaserScan):
        buf = list(msg.ranges)
        f  = self._sm(buf, -20,   20)
        lf = self._sm(buf,  60,  120)
        rf = self._sm(buf, -120, -60)
        u, w = self._calc(buf, f, lf, rf)
        u, w = self._cl(u, w)
        cmd = Twist()
        cmd.linear.x  = float(max(-Q1, min(Q1, u)))
        cmd.angular.z = float(max(-2.0, min(2.0, w)))
        self._p0.publish(cmd)
        self._z1 += 1
        if not self._z1 % 32:
            self.get_logger().info(
                f'f={f:.2f}  lf={lf:.2f}  rf={rf:.2f}  u={u:.2f}  w={w:.2f}'
            )

    def avoid_obstacle(self, f, lf, rf):
        """Obstacle avoidance condition logic: steer away from obstacles."""
        if f < Q4:
            # Obstacle detected ahead: trigger reverse or turn
            return True
        if lf < Q3:
            # Obstacle on left: steer right
            return True
        if rf < Q3:
            # Obstacle on right: steer left
            return True
        return False

    def _calc(self, buf, f, lf, rf):
        n = len(buf)
        if not n:
            return Q1, 0.0

        # Obstacle avoidance: update front-obstacle counter
        if f < Q4:
            self._z2 += 1
        else:
            self._z2 = 0

        a, b = n//4, 3*n//4
        fb = buf[a:b]
        m  = len(fb)
        ss = m // Q7
        if not ss:
            return Q1, 0.0

        bd = []
        for i in range(Q7):
            sl = fb[i*ss : i*ss+ss]
            v  = sorted(r for r in sl if not (math.isinf(r) or math.isnan(r)) and r > 0.12)
            bd.append(v[max(0, int(len(v)*0.25)-1)] if v else 8.0)

        bd = self._xd(bd)

        bi, bv = Q7//2, 0.0
        for i in range(Q7):
            nv = bd[i-1] if i > 0 else bd[i]
            np = bd[i+1] if i < Q7-1 else bd[i]
            if nv < 0.30 and np < 0.30:
                continue
            if bd[i] > bv:
                bv, bi = bd[i], i

        if bv == 0.0:
            bi = max(range(Q7), key=lambda i: bd[i])
            bv = bd[bi]

        if bv < 0.4:
            self._z3 += 1
        else:
            self._z3 = 0

        ba = -((bi / Q7) - 0.5) * 180.0

        pi2 = int(((-self._z0 / 180.0) + 0.5) * Q7)
        pi2 = max(0, min(Q7-1, pi2))
        pv  = bd[pi2]

        if bv < pv*1.15 and pv > 0.5:
            ba = self._z0
        self._z0 = ba

        # Obstacle detected ahead persistently: reverse and turn away
        if self._z2 >= Q8:
            sd = 1.0 if ba >= 0 else -1.0
            self._emit(f'REV {"L" if sd>0 else "R"}')
            return Q5, sd*Q2

        br = math.radians(ba)

        # Best opening is too close: stuck or slow
        if self._z3 >= Q9 and bv < 0.4:
            sd = 1.0 if ba >= 0 else -1.0
            self._emit(f'STUCK {"L" if sd>0 else "R"}')
            return 0.0, sd*Q2
        elif bv < 0.4:
            self._emit(f'SLOW {ba:+.0f}')
            u, w = Q6, (1.0 if ba >= 0 else -1.0)*Q2*0.5
        elif abs(ba) > 45:
            u = Q1*0.18
            w = Q2*0.85*(1.0 if ba>0 else -1.0)
        elif abs(ba) > 15:
            t  = (abs(ba)-15.0)/30.0
            sd = 1.0 if ba>0 else -1.0
            u  = Q1*(0.5-0.2*t)
            w  = max(-Q2, min(Q2, sd*Q2*(0.5+0.5*t)))
        else:
            u = Q1
            w = max(-Q2*0.28, min(Q2*0.28, br*1.0))

        # Side obstacle corrections
        if lf < Q3:
            # Obstacle on left: steer right
            w -= math.sqrt((Q3-lf)/Q3)*Q2*0.5
        if rf < Q3:
            # Obstacle on right: steer left
            w += math.sqrt((Q3-rf)/Q3)*Q2*0.5

        w = max(-Q2, min(Q2, w))
        self._emit(f'B={ba:+.0f} d={bv:.1f} u={u:.2f} w={w:+.2f}')
        return u, w

def main():
    rclpy.init()
    nd = _Nx()
    go = True

    def _bye(s, f):
        nonlocal go
        go = False
    signal.signal(signal.SIGINT, _bye)

    while go:
        rclpy.spin_once(nd, timeout_sec=0.1)

    z = Twist()
    [nd._p0.publish(z) or time.sleep(0.05) for _ in range(10)]
    nd.destroy_node()
    rclpy.shutdown()
    print('\nHalted.')


if __name__ == '__main__':
    main()