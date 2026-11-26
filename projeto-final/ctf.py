#!/usr/bin/env python3
"""Rouba-bandeira.

Os dois robos do time rodam este mesmo arquivo. Cada no le a odometria de todos, as
bandeiras e o /game/state, entao os dois decidem de forma coerente sem trocar mensagens.

Defensor ("tampa"): fica parado sobre a propria bandeira, de frente para o invasor mais
proximo. Quem chega perto e segurado antes de alcancar a bandeira. Enquanto o invasor esta
segurado, o defensor o empurra para longe, e ele e segurado de novo ao fim da imunidade.

Atacante: escolhe o rumo testando direcoes contra o alcance dos defensores. Se travar,
entra aceitando ser segurado e avanca durante a imunidade. Com a bandeira, foge para a
nossa metade; se vai ser segurado, fica sobre o ponto da bandeira ("abraco").

A logica e escrita com a nossa base em x < 0. No time vermelho, x e espelhado na entrada
e o giro troca de sinal na saida.

v1 (robusta a ruido): a odometria de todos os robos passa por um filtro de Kalman estendido
(modelo uniciclo, ruido estimado on-line); a pose usada na decisao e a do filtro prevista
LEAD s a frente (substitui a extrapolacao PRED). A bandeira inimiga nao vem da medida: pelas
regras ela so esta na base ou com o portador, entao usamos a base ou a pose filtrada dele.
"""
import collections
import json
import math
import threading
import time  # relogio monotonico: o de parede (rospy.get_time) pode saltar (ajuste de horario/NTP; no WSL salta 7 s)

try:  # o fastsim.py importa este arquivo sem ROS
    import rospy
    from geometry_msgs.msg import PointStamped, Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
except ImportError:
    rospy = None

import numpy as np

V_MAX = 0.2
W_MAX = 4.0
IMMUNE_TIME = 1.5   # s de imunidade depois de solto (regra do juiz)
WALL = 0.8 - 0.037  # limite do centro do robo na arena
F = (-0.62, 0.0)    # nossa bandeira
E = (0.62, 0.0)     # bandeira deles

K_W = 5.0                      # rad/s por rad (satura em W_MAX)
TURN_FIRST = math.radians(60)  # com erro maior que isso, gira no lugar antes de andar
PRED = 0.0                     # (v0: extrapolacao; na v1 o filtro preve, ver LEAD)
LEAD = 0.06                    # s de atraso compensados pela predicao do filtro

# defesa
SHIFT = 0.012  # m, deslocamento da tampa na direcao do invasor
PIN = True     # empurrar para longe o invasor segurado
PIN_R = 0.35   # m da bandeira, ate onde o empurramos

# ataque
ENGAGE = 0.12       # m, defensor deles a menos disso da bandeira: entrar direto
STUCK = 4.0         # s sem se aproximar da bandeira: entrar direto
STUCK_CARRY = 3.0   # s com a bandeira sem avancar: ir reto para casa
HUG = 0.12          # m, defensor a menos disso com a bandeira na mao: abraco
HUG_EPS = 0.015     # m, raio em que paramos sobre o ponto da bandeira (erro do filtro)
N_DIR = 24          # rumos testados pelo planejador
REACH = 0.6         # fracao de V_MAX com que supomos que um defensor em movimento nos alcanca
REACH_STILL = 0.25  # idem, defensor parado
SAFE = 0.14         # m de folga desejada para um defensor
DANGER = 30.0       # peso da falta de folga
TURN_COST = 0.02    # custo por rad de giro
HYST = 0.05         # custo por rad de trocar o rumo escolhido no ciclo anterior


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


wrap_angle = wrap


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def drive(me, tx, ty, v=V_MAX, reverse=True):
    """Vai a (tx, ty) sempre a v, sem desacelerar (licao do sumo). De re se o alvo esta atras."""
    a = wrap(math.atan2(ty - me[1], tx - me[0]) - me[2])
    if reverse and abs(a) > math.pi / 2:
        a = wrap(a - math.pi)
        v = -v
    return (v if abs(a) < TURN_FIRST else 0.0), clamp(K_W * a, -W_MAX, W_MAX)


def face(me, tx, ty):
    a = wrap(math.atan2(ty - me[1], tx - me[0]) - me[2])
    return 0.0, clamp(K_W * a, -W_MAX, W_MAX)


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


class Brain:
    """Decisao de um robo: step() recebe o jogo (nossa base em x < 0) e devolve (v, w)."""

    def __init__(self, robot, role, mates, opps):
        self.robot, self.role, self.mates, self.opps = robot, role, mates, opps
        self.release = {}   # id -> instante em que foi solto
        self.was_held = {}
        self.best = None    # (menor distancia a bandeira deles, instante)
        self.carry = None   # (x do portador, instante)
        self.last_h = None  # rumo escolhido no ciclo anterior
        self.now = 0.0

    def track(self, held):
        for k, h in held.items():
            if self.was_held.get(k) and not h:
                self.release[k] = self.now
            self.was_held[k] = h

    def immune(self, k):
        return self.now - self.release.get(k, -99.0) < IMMUNE_TIME

    def step(self, now, w):
        """w: pose {id: (x, y, yaw, v)}, enemy_flag (x, y), held e carrying {id: bool}."""
        self.now = now
        self.track(w["held"])
        w["pose"] = {k: (x + v * math.cos(a) * PRED, y + v * math.sin(a) * PRED, a, v)
                     for k, (x, y, a, v) in w["pose"].items()}
        me = w["pose"][self.robot]
        if self.role == "defender":
            return self.defend(w, me)
        return self.attack(w, me)

    # --- defesa ------------------------------------------------------------------

    def defend(self, w, me):
        carrier = next((k for k in self.opps if w["carrying"][k]), None)
        if carrier is not None and not w["held"][carrier]:
            # levaram a bandeira: persegue so ate o meio, do outro lado quem e segurado somos nos
            cx, cy = w["pose"][carrier][:2]
            return drive(me, min(cx, -0.05), cy)
        pin = self.pin_target(w, me)
        if pin is not None:
            return drive(me, *w["pose"][pin][:2], reverse=False)
        # tampa: o invasor so chega a 7 cm da bandeira se nos empurrar SHIFT + 4 mm
        k = min(self.opps, key=lambda k: dist(w["pose"][k], F))
        o = w["pose"][k][:2]
        d = max(dist(o, F), 1e-6)
        t = (F[0] + SHIFT * (o[0] - F[0]) / d, F[1] + SHIFT * (o[1] - F[1]) / d)
        return self.guard(me, t, o, not w["held"][k])

    def pin_target(self, w, me):
        """Invasor segurado (ou recem-solto) encostado em nos, perto da bandeira, sem outro
        adversario solto por perto. Empurrando-o enquanto esta segurado, o momento da dupla e
        nosso quando ele e solto, e ele e segurado de novo ao fim da imunidade."""
        if not PIN:
            return None
        cand = [k for k in self.opps if (w["held"][k] or self.immune(k)) and dist(w["pose"][k], me) < 0.12
                and dist(w["pose"][k], F) < PIN_R and w["pose"][k][0] < -0.15]
        others = [k for k in self.opps if k not in cand and not w["held"][k] and dist(w["pose"][k], F) < 0.6]
        if not cand or others:
            return None
        return min(cand, key=lambda k: dist(w["pose"][k], me))

    def guard(self, me, t, o, live):
        """Fica no ponto t de frente para o invasor o; se ele encosta, empurra de volta a toda."""
        ex, ey = t[0] - me[0], t[1] - me[1]
        if math.hypot(ex, ey) > 0.03:
            return drive(me, *t)
        _, om = face(me, *o)
        if live and dist(me, o) < 0.09:
            return V_MAX, om
        along = ex * math.cos(me[2]) + ey * math.sin(me[2])
        return clamp(10.0 * along, -V_MAX, V_MAX), om

    # --- ataque ------------------------------------------------------------------

    def attack(self, w, me):
        imm = max(0.0, IMMUNE_TIME - (self.now - self.release.get(self.robot, -99.0)))
        if w["carrying"][self.robot]:
            self.best = None
            if me[0] <= 0.0:  # na nossa metade ninguem nos segura
                return drive(me, F[0] + 0.09, 0.06 if me[1] >= 0 else -0.06)
            gd = min([dist(w["pose"][k], me) for k in self.guards(w)] or [9.0])
            if not imm and gd < HUG and dist(me, E) < 0.12:
                # abraco: segurados sobre o ponto da bandeira, ela volta para a base e o juiz
                # nos da a bandeira de novo no mesmo ciclo
                return (0.0, 0.0) if dist(me, E) < HUG_EPS else drive(me, *E)
            if self.carry is None or me[0] < self.carry[0] - 0.05:
                self.carry = (me[0], self.now)
            if self.now - self.carry[1] > STUCK_CARRY:
                return drive(me, -0.05, me[1])
            return self.plan(w, me, lambda p: max(p[0] + 0.06, 0.0) + 0.2 * dist(p, F), imm)

        self.carry = None
        flag = w["enemy_flag"]
        d = dist(me, flag)
        if me[0] < 0.0 or self.best is None or d < self.best[0] - 0.05:
            self.best = (d, self.now)
        # defensor plantado na bandeira ou travado: entra direto; se for segurado, avanca na imunidade
        if any(dist(w["pose"][k], E) < ENGAGE for k in self.guards(w)) or self.now - self.best[1] > STUCK:
            return drive(me, *flag, reverse=False)
        return self.plan(w, me, lambda p: dist(p, flag), imm)

    def guards(self, w):
        """Adversarios que podem nos segurar: soltos e na metade deles."""
        return [k for k in self.opps if not w["held"][k] and w["pose"][k][0] > -0.03]

    def plan(self, w, me, cost, imm):
        """Testa N_DIR rumos: onde estaremos em 0,4 s e 0,8 s contra onde os defensores podem estar."""
        gs = [(w["pose"][k][:2], REACH if abs(w["pose"][k][3]) > 0.02 else REACH_STILL) for k in self.guards(w)]
        best, bh = 1e9, 0.0
        for i in range(N_DIR):
            h = me[2] + 2.0 * math.pi * i / N_DIR
            pen = 0.0
            for T in (0.4, 0.8):
                p = (clamp(me[0] + V_MAX * T * math.cos(h), -WALL, WALL),
                     clamp(me[1] + V_MAX * T * math.sin(h), -WALL, WALL))
                if p[0] > 0.0 and T > imm:  # imune ou na nossa metade: sem risco
                    for g, reach in gs:
                        m = dist(g, p) - reach * V_MAX * T
                        if m < SAFE:
                            pen += (SAFE - m) * DANGER
            a = abs(wrap(h - me[2]))
            s = cost(p) + pen + TURN_COST * min(a, math.pi - a)  # de re conta como de frente
            if self.last_h is not None:
                s += HYST * abs(wrap(h - self.last_h))
            if s < best:
                best, bh = s, h
        self.last_h = bh
        return drive(me, me[0] + math.cos(bh), me[1] + math.sin(bh))


def team_frame(s, pose, enemy_flag, game):
    """Mundo -> nossa base em x < 0 (s = -1 espelha x para o time vermelho)."""
    return {
        "pose": {k: (s * x, y, yaw if s > 0 else wrap(math.pi - yaw), v) for k, (x, y, yaw, v) in pose.items()},
        "enemy_flag": (s * enemy_flag[0], enemy_flag[1]),
        "held": {k: bool(h) for k, h in enumerate(game["held"])},
        "carrying": {k: bool(c) for k, c in enumerate(game["carrying"])},
    }


class Core:
    """Um robo do time sem ROS: on_odom/on_flag/on_game recebem as medidas, control(t)
    devolve {robo: (v, w)} no referencial do mundo."""

    def __init__(self, robot, team, role, opps, brain_cls=Brain):
        self.robot, self.team = robot, team
        self.opps = list(opps)
        self.mates = [k for k in range(2 * len(self.opps)) if k not in self.opps]
        self.s = 1.0 if team == "blue" else -1.0
        self.brain = brain_cls(robot, role, self.mates, self.opps)
        self.noise = Noise()
        self.tracks = {k: Track(self.noise, own=(k == robot)) for k in self.mates + self.opps}
        self.flag, self.game = {}, None
        self.lock = threading.Lock()

    def on_odom(self, k, t, x, y, yaw, v, w, sig=None):
        if k in self.tracks:
            with self.lock:
                self.tracks[k].update(t, x, y, yaw, v, w, sig)

    def on_flag(self, color, x, y):
        self.flag[color] = (x, y)

    def on_game(self, game):
        self.game = game

    def enemy_flag(self, pose, game):
        """Bandeira inimiga no mundo: com o nosso portador (um pouco atras do centro) ou na base."""
        carrying = game.get("carrying")
        if carrying is None:
            return self.flag.get("red" if self.team == "blue" else "blue")
        for k in self.mates:
            if k < len(carrying) and carrying[k]:
                x, y, a, _ = pose[k]
                return (x - 0.012 * math.cos(a), y - 0.012 * math.sin(a))
        return (self.s * game.get("base_x", E[0]), 0.0)

    def control(self, t):
        game = self.game
        if not game:
            return {}
        with self.lock:
            st = {k: tr.get(t + LEAD) for k, tr in self.tracks.items()}
        if any(v is None for v in st.values()):
            return {}
        pose = {k: (x, y, a, v) for k, (x, y, a, v, _) in st.items()}
        flag = self.enemy_flag(pose, game)
        if flag is None:
            return {}
        v, om = self.brain.step(t, team_frame(self.s, pose, flag, game))
        held = game.get("held", [])
        with self.lock:  # segurado: o robo nao anda, o modelo do filtro deve saber
            if self.robot < len(held) and held[self.robot]:
                self.tracks[self.robot].set_cmd(0.0, 0.0)
            else:
                self.tracks[self.robot].set_cmd(v, self.s * om)
        return {self.robot: (v, self.s * om)}


class Node:
    def __init__(self, brain_cls=Brain):
        rospy.init_node("ctf")
        robot = rospy.get_param("~robot", 0)
        team = rospy.get_param("~team", "blue" if robot == 0 else "red")
        role = rospy.get_param("~role", "attacker")
        opps = list(rospy.get_param("~opponents", [1 if robot == 0 else 0]))
        self.core = Core(robot, team, role, opps, brain_cls)
        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % robot, Twist, queue_size=1)
        for k in self.core.mates + self.core.opps:
            rospy.Subscriber("/hero_%d/odom" % k, Odometry, self.on_odom, callback_args=k, queue_size=1)
        for c in ("blue", "red"):
            rospy.Subscriber("/flag/" + c, PointStamped, self.on_flag, callback_args=c, queue_size=1)
        rospy.Subscriber("/game/state", String, self.on_game, queue_size=1)

    def on_odom(self, msg, k):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        tw = msg.twist.twist
        self.core.on_odom(k, time.monotonic(), p.x, p.y, yaw, tw.linear.x, tw.angular.z, sigmas_from_odom(msg))

    def on_flag(self, msg, c):
        self.core.on_flag(c, msg.point.x, msg.point.y)

    def on_game(self, msg):
        self.core.on_game(json.loads(msg.data))

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            for i, (v, om) in self.core.control(time.monotonic()).items():
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = clamp(v, -V_MAX, V_MAX), clamp(om, -W_MAX, W_MAX)
                self.pub.publish(cmd)
            rate.sleep()


if __name__ == "__main__":
    Node().run()
