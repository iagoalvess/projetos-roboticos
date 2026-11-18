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
"""
import json
import math

try:  # o fastsim.py importa este arquivo sem ROS
    import rospy
    from geometry_msgs.msg import PointStamped, Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from psr_hero.utils import pose_from_odom
except ImportError:
    rospy = None

V_MAX = 0.2
W_MAX = 4.0
IMMUNE_TIME = 1.5   # s de imunidade depois de solto (regra do juiz)
WALL = 0.8 - 0.037  # limite do centro do robo na arena
F = (-0.62, 0.0)    # nossa bandeira
E = (0.62, 0.0)     # bandeira deles

K_W = 5.0                      # rad/s por rad (satura em W_MAX)
TURN_FIRST = math.radians(60)  # com erro maior que isso, gira no lugar antes de andar
PRED = 0.06                    # s de atraso da odometria compensados por extrapolacao

# defesa
SHIFT = 0.012  # m, deslocamento da tampa na direcao do invasor
PIN = True     # empurrar para longe o invasor segurado
PIN_R = 0.35   # m da bandeira, ate onde o empurramos

# ataque
ENGAGE = 0.12       # m, defensor deles a menos disso da bandeira: entrar direto
STUCK = 4.0         # s sem se aproximar da bandeira: entrar direto
STUCK_CARRY = 3.0   # s com a bandeira sem avancar: ir reto para casa
HUG = 0.12          # m, defensor a menos disso com a bandeira na mao: abraco
N_DIR = 24          # rumos testados pelo planejador
REACH = 0.6         # fracao de V_MAX com que supomos que um defensor em movimento nos alcanca
REACH_STILL = 0.25  # idem, defensor parado
SAFE = 0.14         # m de folga desejada para um defensor
DANGER = 30.0       # peso da falta de folga
TURN_COST = 0.02    # custo por rad de giro
HYST = 0.05         # custo por rad de trocar o rumo escolhido no ciclo anterior


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


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
                return (0.0, 0.0) if dist(me, E) < 0.01 else drive(me, *E)
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


class Node:
    def __init__(self, brain_cls=Brain):
        rospy.init_node("ctf")
        robot = rospy.get_param("~robot", 0)
        self.team = rospy.get_param("~team", "blue" if robot == 0 else "red")
        role = rospy.get_param("~role", "attacker")
        opps = list(rospy.get_param("~opponents", [1 if robot == 0 else 0]))
        mates = [k for k in range(2 * len(opps)) if k not in opps]
        self.s = 1.0 if self.team == "blue" else -1.0
        self.brain = brain_cls(robot, role, mates, opps)
        self.n = len(mates) + len(opps)
        self.pose, self.flag, self.game = {}, {}, None
        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % robot, Twist, queue_size=1)
        for k in mates + opps:
            rospy.Subscriber("/hero_%d/odom" % k, Odometry, self.on_odom, callback_args=k, queue_size=1)
        for c in ("blue", "red"):
            rospy.Subscriber("/flag/" + c, PointStamped, self.on_flag, callback_args=c, queue_size=1)
        rospy.Subscriber("/game/state", String, self.on_game, queue_size=1)

    def on_odom(self, msg, k):
        x, y, yaw = pose_from_odom(msg)
        self.pose[k] = (x, y, yaw, msg.twist.twist.linear.x)

    def on_flag(self, msg, c):
        self.flag[c] = (msg.point.x, msg.point.y)

    def on_game(self, msg):
        self.game = json.loads(msg.data)

    def run(self):
        rate = rospy.Rate(20)
        enemy = "red" if self.team == "blue" else "blue"
        while not rospy.is_shutdown():
            if len(self.pose) == self.n and enemy in self.flag and self.game:
                w = team_frame(self.s, self.pose, self.flag[enemy], self.game)
                v, om = self.brain.step(rospy.get_time(), w)
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = v, self.s * om
                self.pub.publish(cmd)
            rate.sleep()


if __name__ == "__main__":
    Node().run()
