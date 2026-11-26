#!/usr/bin/env python3
"""Sumo: empurra sempre a V_MAX, de frente para o adversario.

No simulador, quem comanda menos velocidade no contato perde o empurrao
(ver teste_fisica.py). Quando a briga trava, o robo empurra com o rumo
desviado para arrastar os dois para a borda do adversario.

v2 (robusta a ruido): a logica de decisao e a da v0, mas a odometria dos dois robos passa
por um filtro de Kalman estendido (modelo uniciclo; no nosso robo o comando publicado entra
no modelo). O desvio-padrao de cada medida vem da covariancia que o bridge publica na
Odometry (kit com ruido); sem ela, e estimado on-line. A decisao usa as poses previstas LEAD s
a frente. No Genesis com o ruido do kit v2, sem filtro todas as rodadas contra a v0 empatam;
com filtro, 22 x 0.
"""
import collections
import math
import threading
import time  # relogio monotonico: o de parede (rospy.get_time) pode saltar (ajuste de horario/NTP; no WSL salta 7 s)

try:  # o testes_ruido/fastsim2d.py importa este arquivo sem ROS
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
except ImportError:
    rospy = None

import numpy as np

V_MAX = 0.2
W_MAX = 4.0

RING_RADIUS = 0.40
R_HARD = 0.35       # m, sempre volta para o centro
R_GUARD = 0.28      # m, volta se estiver solto e mais perto da borda que o adversario
K_W = 5.0           # rad/s por rad
TURN_FIRST = math.radians(60)
CONTACT = 0.085     # m entre centros
STALL_SPEED = 0.03  # m/s
STALL_TIME = 1.0    # s
DRAG = math.radians(20)
LEAD = 0.04         # s de atraso (transporte + atuacao) compensados pela predicao do filtro


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# === estimador de estado (o mesmo bloco em sumo.py, ctf.py e soccer.py) ===================
TAU_V, TAU_W = 0.07, 0.04          # s, resposta do robo ao comando (forca e torque limitados)
Q_ACC, Q_ALPHA = 0.25, 5.0        # m/s^1.5, rad/s^1.5: aceleracao nao modelada (outros robos)
Q_ACC_OWN, Q_ALPHA_OWN = 0.25, 5.0  # idem no nosso robo (comando conhecido)
Q_SLIP = 0.006                     # m/s^0.5: deslizamento lateral (empurroes); afinado com o ruido do kit v2
GATE = 0.06                        # m: salto menor que isso nunca e outlier
GATE_SIGMAS = 5.0                  # inovacao acima de 5 sigma (2 seguidas) = teleporte (nova rodada)
TELEPORT = 0.20                    # m: salto deste tamanho e acima de 10 sigma reinicia o filtro na hora


def sigmas_from_odom(msg):
    """(sx, sy, syaw, sv, sw) que o bridge publica na covariancia da Odometry (kit com ruido),
    ou None se ela vier zerada (kit sem ruido: o filtro estima o ruido sozinho)."""
    try:
        pc, tc = msg.pose.covariance, msg.twist.covariance
        if len(pc) < 36 or pc[0] <= 0.0:
            return None
        return (math.sqrt(pc[0]), math.sqrt(pc[7]), math.sqrt(max(pc[35], 0.0)),
                math.sqrt(max(tc[0], 0.0)), math.sqrt(max(tc[35], 0.0)))
    except (AttributeError, TypeError, ValueError):
        return None


class Noise:
    """Desvio-padrao do ruido de cada canal (p: x/y, a: yaw, v, w), estimado pela mediana do
    modulo da 2a diferenca das medidas cruas: sigma = med / (0,6745 sqrt 6). A mediana ignora
    teleportes e manobras; o piso evita confiar demais quando nao ha ruido."""
    FLOOR = {"p": 0.0015, "a": 0.004, "v": 0.004, "w": 0.04}
    CEIL = {"p": 0.10, "a": 0.8, "v": 0.2, "w": 3.0}

    def __init__(self, n=150):
        self.buf = {k: collections.deque(maxlen=n) for k in self.FLOOR}
        self.sigma = {"p": 0.01, "a": 0.05, "v": 0.02, "w": 0.3}
        self.count = 0

    def add(self, k, d2):
        self.buf[k].append(abs(d2))
        self.count += 1
        if self.count % 20 == 0:
            for c, b in self.buf.items():
                if len(b) >= 30:
                    s = sorted(b)[len(b) // 2] / 1.652
                    self.sigma[c] = clamp(s, self.FLOOR[c], self.CEIL[c])


class Track:
    """EKF de um robo, estado [x, y, yaw, v, w]. Mede [x, y, yaw] e [v, w] (twist).
    own=True: o comando publicado (set_cmd) e a entrada do modelo de v e w."""

    def __init__(self, noise, own=False):
        self.noise, self.own = noise, own
        self.s = None
        self.P = None
        self.t = 0.0
        self.cmd = (0.0, 0.0)
        self.raw = collections.deque(maxlen=3)
        self.bad = 0

    def set_cmd(self, v, w):
        self.cmd = (v, w)

    def _reset(self, t, z):
        sp, sa = self.noise.sigma["p"], self.noise.sigma["a"]
        self.s = np.array(z, dtype=float)
        self.P = np.diag([sp * sp, sp * sp, sa * sa, 0.05 ** 2, 1.0])
        self.t, self.bad = t, 0

    def _step(self, s, P, dt):
        x, y, a, v, w = s
        c, si = math.cos(a), math.sin(a)
        if self.own:
            kv, kw = math.exp(-dt / TAU_V), math.exp(-dt / TAU_W)
            v2 = self.cmd[0] + (v - self.cmd[0]) * kv
            w2 = self.cmd[1] + (w - self.cmd[1]) * kw
            qa, qw = Q_ACC_OWN, Q_ALPHA_OWN
        else:
            kv = kw = 1.0
            v2, w2 = v, w
            qa, qw = Q_ACC, Q_ALPHA
        s2 = np.array([x + v * c * dt, y + v * si * dt, a + w * dt, v2, w2])
        F = np.eye(5)
        F[0, 2], F[0, 3] = -v * si * dt, c * dt
        F[1, 2], F[1, 3] = v * c * dt, si * dt
        F[2, 4], F[3, 3], F[4, 4] = dt, kv, kw
        q = Q_SLIP * Q_SLIP * dt
        Q = np.diag([q, q, 1e-5 * dt, qa * qa * dt, qw * qw * dt])
        return s2, F @ P @ F.T + Q

    def _predict(self, s, P, t0, t1):
        while t1 - t0 > 1e-6:
            dt = min(0.02, t1 - t0)
            s, P = self._step(s, P, dt)
            t0 += dt
        return s, P

    def update(self, t, x, y, yaw, v, w, sig=None):
        """sig: (sx, sy, syaw, sv, sw) publicados pelo bridge; sem eles, usa a estimativa on-line."""
        nz = self.noise
        self.raw.append((x, y, yaw, v, w))
        if len(self.raw) == 3:
            (x0, y0, a0, v0, w0), (x1, y1, a1, v1, w1), _ = self.raw
            nz.add("p", x - 2 * x1 + x0)
            nz.add("p", y - 2 * y1 + y0)
            nz.add("a", wrap_angle(yaw - a1) - wrap_angle(a1 - a0))
            nz.add("v", v - 2 * v1 + v0)
            nz.add("w", w - 2 * w1 + w0)
        fl = Noise.FLOOR
        if sig is not None:
            sx, sy = max(sig[0], fl["p"]), max(sig[1], fl["p"])
            sa, sv, sw = max(sig[2], fl["a"]), max(sig[3], fl["v"]), max(sig[4], fl["w"])
        else:
            sx = sy = nz.sigma["p"]
            sa, sv, sw = nz.sigma["a"], nz.sigma["v"], nz.sigma["w"]
        if self.s is None:
            self._reset(t, (x, y, yaw, v, w))
            self.P[0, 0], self.P[1, 1], self.P[2, 2] = sx * sx, sy * sy, sa * sa
            return
        s, P = self._predict(self.s, self.P, self.t, max(t, self.t))
        # pose: x, y, yaw
        nu = np.array([x - s[0], y - s[1], wrap_angle(yaw - s[2])])
        S = P[:3, :3] + np.diag([sx * sx, sy * sy, sa * sa])
        jump = math.hypot(nu[0], nu[1])
        d2 = nu[0] * nu[0] / S[0, 0] + nu[1] * nu[1] / S[1, 1]  # inovacao em sigmas^2
        if jump > GATE and d2 > GATE_SIGMAS ** 2:
            self.bad += 1
            if self.bad >= 2 or (jump > TELEPORT and d2 > 4 * GATE_SIGMAS ** 2):  # teleporte
                self._reset(t, (x, y, yaw, v, w))
                self.P[0, 0], self.P[1, 1], self.P[2, 2] = sx * sx, sy * sy, sa * sa
            else:
                self.s, self.P, self.t = s, P, max(t, self.t)
            return
        self.bad = 0
        K = P[:, :3] @ np.linalg.inv(S)
        s = s + K @ nu
        P = P - K @ P[:3, :]
        # twist: v, w
        nu = np.array([v - s[3], w - s[4]])
        S = P[3:, 3:] + np.diag([sv * sv, sw * sw])
        K = P[:, 3:] @ np.linalg.inv(S)
        s = s + K @ nu
        P = P - K @ P[3:, :]
        s[2] = wrap_angle(s[2])
        self.s, self.P, self.t = s, 0.5 * (P + P.T), max(t, self.t)

    def get(self, t):
        """(x, y, yaw, v, w) previsto para o instante t."""
        if self.s is None:
            return None
        s, _ = self._predict(self.s, self.P, self.t, max(t, self.t))
        return float(s[0]), float(s[1]), wrap_angle(float(s[2])), float(s[3]), float(s[4])
# === fim do estimador ======================================================================


class Core:
    """Decisao sem ROS: on_odom() recebe as medidas, control(t) devolve {robo: (v, w)}."""

    def __init__(self, robot, opponent):
        self.robot, self.opponent = robot, opponent
        self.noise = Noise()
        self.tracks = {robot: Track(self.noise, own=True), opponent: Track(self.noise)}
        self.stall_since = None
        self.drag_side = 0.0
        self.lock = threading.Lock()

    def on_odom(self, k, t, x, y, yaw, v, w, sig=None):
        if k in self.tracks:
            with self.lock:
                self.tracks[k].update(t, x, y, yaw, v, w, sig)

    def control(self, t):
        with self.lock:
            me = self.tracks[self.robot].get(t + LEAD)
            opp = self.tracks[self.opponent].get(t + LEAD)
        if me is None or opp is None:
            return {}
        v, w = self.compute_command(me, opp, t)
        with self.lock:
            self.tracks[self.robot].set_cmd(v, w)
        return {self.robot: (v, w)}

    def head_to(self, me, tx, ty, offset=0.0):
        a = wrap_angle(math.atan2(ty - me[1], tx - me[0]) + offset - me[2])
        v = V_MAX if abs(a) < TURN_FIRST else 0.0
        return v, clamp(K_W * a, -W_MAX, W_MAX)

    def back_to_center(self, me):
        x, y, yaw = me[:3]
        a = wrap_angle(math.atan2(-y, -x) - yaw)
        if abs(a) > math.pi / 2:  # centro atras: volta de re
            a = wrap_angle(a - math.pi)
            return -V_MAX, clamp(K_W * a, -W_MAX, W_MAX)
        return V_MAX, clamp(K_W * a, -W_MAX, W_MAX)

    def compute_command(self, me, opp, now):
        x, y = me[:2]
        ox, oy = opp[:2]
        speed, opp_speed = me[3], opp[3]
        r_me, r_opp = math.hypot(x, y), math.hypot(ox, oy)
        dx, dy = ox - x, oy - y
        d = max(math.hypot(dx, dy), 1e-6)

        if r_me > R_HARD or (d > CONTACT + 0.02 and r_me > R_GUARD and r_me > r_opp - 0.02):
            self.stall_since, self.drag_side = None, 0.0
            return self.back_to_center(me)

        # briga travada: o lado do arrasto e escolhido uma vez por contato
        if d > CONTACT + 0.015 or abs(speed) > 2 * STALL_SPEED:
            self.stall_since, self.drag_side = None, 0.0
        elif self.drag_side == 0.0:
            if abs(speed) < STALL_SPEED and abs(opp_speed) < STALL_SPEED:
                if self.stall_since is None:
                    self.stall_since = now
                elif now - self.stall_since > STALL_TIME:
                    self.drag_side = 1.0 if (-dy * ox + dx * oy) >= 0 else -1.0
            else:
                self.stall_since = None
        if self.drag_side != 0.0:
            return self.head_to(me, ox, oy, self.drag_side * DRAG)
        return self.head_to(me, ox, oy)


class Sumo:
    def __init__(self):
        rospy.init_node("sumo")
        self.robot = rospy.get_param("~robot", 0)
        self.opponent = rospy.get_param("~opponent", 1)
        self.core = Core(self.robot, self.opponent)
        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % self.robot, Twist, queue_size=1)
        for k in (self.robot, self.opponent):
            rospy.Subscriber("/hero_%d/odom" % k, Odometry, self.on_odom, callback_args=k, queue_size=1)

    def on_odom(self, msg, k):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        tw = msg.twist.twist
        self.core.on_odom(k, time.monotonic(), p.x, p.y, yaw, tw.linear.x, tw.angular.z, sigmas_from_odom(msg))

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            for i, (v, w) in self.core.control(time.monotonic()).items():
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = clamp(v, -V_MAX, V_MAX), clamp(w, -W_MAX, W_MAX)
                self.pub.publish(cmd)
            rate.sleep()


if __name__ == "__main__":
    Sumo().run()
