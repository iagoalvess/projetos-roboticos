#!/usr/bin/env python3
"""Futebol de robos: um no controla os tres robos do time.

A logica e escrita com o nosso gol em x = -0,9 e o ataque para +x. No time vermelho, x e
espelhado na entrada e o giro troca de sinal na saida (como no rouba-bandeira).

O que o teste de fisica (teste_fisica.py) mostrou e decidiu a estrategia:
- a batida e inelastica: a bola sai com a velocidade do robo (0,2 m/s) e fica na frente
  dele enquanto ele empurra reto. Nao ha chute: o que vale e conduzir alinhado;
- girar com a bola na frente perde a bola; bater fora do centro desvia (4,5 cm -> 45 graus);
- a bola nao quica na parede e desliza rente a ela quando empurrada contra ela;
- quem empurra a bola contra um robo parado leva os dois a 0,1 m/s; contra um robo que
  empurra de volta, trava. Dois robos nossos em fila empurram com o dobro da forca.

Papeis (decididos a cada ciclo, com histerese):
- goleiro: na linha, no ponto onde a bola cruzaria; se a bola encosta, empurra de volta
  para fora, de lado, nunca parado;
- atacante: o jogador que chega antes atras da bola; contorna a bola, alinha na linha
  bola-gol (mirando o canto livre) e conduz a 0,2 m/s;
- apoio: o outro jogador. SUPPORT escolhe o que ele faz (ver support()).
"""
import json
import math

try:  # o fastsim.py importa este arquivo sem ROS
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
except ImportError:
    rospy = None

V_MAX = 0.2
W_MAX = 4.0
R = 0.037           # raio do robo
RB = 0.0214         # raio da bola
TOUCH = R + RB      # centro do robo ao centro da bola, encostados
HALF_L, HALF_W = 0.9, 0.6
GOAL_W = 0.40
X_LIM, Y_LIM = HALF_L - R, HALF_W - R  # onde o centro do robo cabe

K_W = 5.0
TURN_FIRST = math.radians(60)
PRED = 0.06         # s de atraso da odometria compensados por extrapolacao

# atacante
AIM_Y = 0.12        # alvos testados dentro do gol: -AIM_Y, 0, +AIM_Y
ALIGN = 0.022       # m de erro lateral aceito para conduzir
ALIGN_ANG = math.radians(25)
BEHIND = 0.09       # m atras da bola onde o atacante se alinha
AROUND = 0.11       # m de folga ao contornar a bola
LOOK = 0.15         # s de antecipacao da bola
SWITCH = 0.12       # m de vantagem para trocar de atacante
DRIBBLE_STEP = 0.0  # graus entre os rumos do leque (11 rumos)
CORRIDOR = 0.3      # m do corredor de conducao que precisa estar livre
STALL = 0.5         # s com a bola parada e encostada: travou
STALL_ANG = 60.0    # graus de desvio para destravar
STALL_FOR = 1.0     # s empurrando torto

# apoio
SUPPORT = "bloqueio" # trem | zaga | bloqueio | sequestro
TRAIN_GAP = 2 * R + 0.004
ZAGA_X = -0.45

STUCK = 3.0         # s com a bola parada: tirar o adversario que a prende

LOCK = 1            # vantagem no placar a partir da qual trancamos a bola no canto (99 = nunca)

# goleiro
LIBERO_X = 9.0      # bola alem disso (no campo deles): o goleiro sobe ate LIBERO_TO (9 = nunca)
LIBERO_TO = -0.45
GK_X = -HALF_L + R + 0.01
GK_Y = GOAL_W / 2 - 0.035
KEEPER = "lado_afasta"  # frente (nosso) | lado | lado_afasta (goleiro do Lucas, sem / com afastar)
SIDE_X = -0.89      # linha do goleiro "de lado"
CLEAR_R = 0.10      # bola a menos disso do goleiro, na frente dele: afasta

# placar
WALL_LEAD = 99      # vantagem a partir da qual o apoio vira segundo goleiro (99 = nunca)


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def unit(dx, dy):
    d = math.hypot(dx, dy)
    return (dx / d, dy / d) if d > 1e-9 else (1.0, 0.0)


def inside(p):
    return clamp(p[0], -X_LIM, X_LIM), clamp(p[1], -Y_LIM, Y_LIM)


def drive(me, tx, ty, v=V_MAX, reverse=True):
    """Vai a (tx, ty) sempre a v, sem desacelerar. De re se o alvo esta atras (o robo e redondo)."""
    a = wrap(math.atan2(ty - me[1], tx - me[0]) - me[2])
    if reverse and abs(a) > math.pi / 2:
        a = wrap(a - math.pi)
        v = -v
    return (v if abs(a) < TURN_FIRST else 0.0), clamp(K_W * a, -W_MAX, W_MAX)


def arrive(me, tx, ty):
    """Como drive, mas freia perto do alvo (para ficar parado num ponto)."""
    d = math.hypot(tx - me[0], ty - me[1])
    v, w = drive(me, tx, ty)
    return v * clamp(d / 0.04, 0.0, 1.0), w


def seg_clear(a, b, p):
    """Distancia do ponto p ao segmento a-b."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = clamp(((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / max(dx * dx + dy * dy, 1e-9), 0.0, 1.0)
    return math.hypot(a[0] + t * dx - p[0], a[1] + t * dy - p[1])


class Brain:
    """step() recebe o jogo (nosso gol em x < 0) e devolve [(v, w)] para self.ids."""

    def __init__(self, ids, opps):
        self.ids, self.opps = ids, opps
        self.keeper = ids[0]
        self.striker = ids[1]
        self.now = 0.0
        self.last_a = None  # rumo de conducao do ciclo anterior
        self.stall_t = None
        self.stuck_t = None
        self.push = ((0.0, 0.0), (1.0, 0.0))  # (bola, rumo) da conducao do atacante

    def step(self, now, w):
        """w: pose {id: (x, y, yaw, v)}, ball (x, y, vx, vy), score (nos, eles)."""
        self.now = now
        P = {k: (x + v * math.cos(a) * PRED, y + v * math.sin(a) * PRED, a, v) for k, (x, y, a, v) in w["pose"].items()}
        bx, by, bvx, bvy = w["ball"]
        ball = (bx + bvx * PRED, by + bvy * PRED)
        self.P, self.ball, self.bv = P, ball, (bvx, bvy)
        self.lead = w["score"][0] - w["score"][1]
        field = [k for k in self.ids if k != self.keeper]
        cost = {k: self.reach_cost(P[k]) for k in field}
        other = next(k for k in field if k != self.striker)
        if cost[other] < cost[self.striker] - SWITCH:
            self.striker, self.last_a, self.stall_t = other, None, None
        helper = next(k for k in field if k != self.striker)
        out = {self.keeper: self.keep(P[self.keeper]),
               self.striker: self.attack(P[self.striker]),
               helper: self.support(P[helper], P[self.striker])}
        return [out[k] for k in self.ids]

    # --- ataque ------------------------------------------------------------------

    def aim(self, ball):
        """Ponto do gol deles mais longe dos adversarios, olhando a linha bola-alvo."""
        best, bg = -1.0, (HALF_L + 0.05, 0.0)
        for gy in (-AIM_Y, 0.0, AIM_Y):
            g = (HALF_L + 0.05, gy)
            c = min(seg_clear(ball, g, self.P[k]) for k in self.opps) - 0.01 * abs(gy - ball[1])
            if c > best:
                best, bg = c, g
        return bg

    def lead_ball(self, me):
        """Onde a bola estara quando chegarmos (ela freia a ~0,08 m/s2)."""
        t = LOOK + dist(me, self.ball) / V_MAX * 0.5
        vx, vy = self.bv
        s = max(0.0, 1.0 - 0.4 * t / max(math.hypot(vx, vy), 0.05))
        return inside((self.ball[0] + vx * t * s, self.ball[1] + vy * t * s))

    def reach_cost(self, me):
        b = self.lead_ball(me)
        g = self.corner(b) if self.lead >= LOCK else self.aim(b)
        u = unit(g[0] - b[0], g[1] - b[1])
        along = (me[0] - b[0]) * u[0] + (me[1] - b[1]) * u[1]
        return dist(me, (b[0] - BEHIND * u[0], b[1] - BEHIND * u[1])) + (0.25 if along > -0.02 else 0.0)

    def corner(self, b):
        """Canto da mesma metade e do mesmo lado da bola. Bola no canto nao sai mais: nenhum robo
        alcanca o lado de dentro dela (teste_fisica.py, testes canto_*)."""
        return (math.copysign(HALF_L - RB, b[0] if abs(b[0]) > 0.05 else 1.0),
                math.copysign(HALF_W - RB, b[1] if abs(b[1]) > 0.02 else 1.0))

    def push_dir(self, b, me):
        """Rumo de conducao: o leque de +-75 graus em volta do alvo no gol; perde pontos o rumo cujo
        corredor (0,3 m) passa por um adversario ou sai do campo. A bola encostada num adversario
        fora do centro escorrega em volta dele (atrito 0,1): e o nosso drible."""
        if self.lead >= LOCK:  # na frente: leva a bola para o canto e o jogo congela
            c = self.corner(b)
            return unit(c[0] - b[0], c[1] - b[1])
        g = self.aim(b)
        g_ang = math.atan2(g[1] - b[1], g[0] - b[0])
        if b[0] > HALF_L - 0.25 and abs(b[1]) < GOAL_W / 2:
            return unit(g[0] - b[0], g[1] - b[1])  # na boca do gol: direto
        best, bu = 1e9, None
        for k in range(-5, 6):
            a = g_ang + math.radians(DRIBBLE_STEP) * k
            u = (math.cos(a), math.sin(a))
            end = (b[0] + CORRIDOR * u[0], b[1] + CORRIDOR * u[1])
            pen = 0.0
            for o in self.opps:
                c = seg_clear(b, end, self.P[o])
                if c < R + RB + 0.02:
                    pen += (R + RB + 0.02 - c) * 20.0
            if abs(end[1]) > HALF_W - RB and not (end[0] > HALF_L - 0.1 and abs(end[1]) < GOAL_W / 2):
                pen += 0.3
            cost = -math.cos(a - g_ang) + pen
            if self.last_a is not None:
                cost += 0.15 * abs(wrap(a - self.last_a))
            if cost < best:
                best, bu = cost, a
        self.last_a = bu
        return math.cos(bu), math.sin(bu)

    def attack(self, me):
        b = self.lead_ball(me)
        if self.lead >= LOCK and dist(b, self.corner(b)) < 0.02:
            self.push = (b, (1.0, 0.0))
            return self.support(me, me)  # bola trancada no canto: vira zagueiro
        if math.hypot(*self.bv) > 0.02 or self.stuck_t is None:
            self.stuck_t = self.now if math.hypot(*self.bv) <= 0.02 else None
        elif self.now - self.stuck_t > STUCK:
            # bola presa (canto, prensada) ha STUCK s: tira dali o adversario que a prende
            k = min(self.opps, key=lambda k: dist(self.P[k], b))
            o = self.P[k]
            if dist(o, b) < TOUCH + 0.03:
                u = unit(o[0] - b[0], o[1] - b[1])
                return drive(me, o[0] + 0.1 * u[0], o[1] + 0.1 * u[1])
        u = self.push_dir(b, me)
        self.push = (b, u)
        rx, ry = me[0] - b[0], me[1] - b[1]
        along = rx * u[0] + ry * u[1]
        lat = -rx * u[1] + ry * u[0]
        head = wrap(math.atan2(b[1] - me[1], b[0] - me[0]) - math.atan2(u[1], u[0]))
        # travou (bola quase parada, nos encostados): empurra torto por STALL_FOR s, como no sumo
        if dist(me, b) < TOUCH + 0.015 and math.hypot(*self.bv) < 0.03:
            if self.stall_t is None:
                self.stall_t = self.now
        elif self.stall_t is not None and self.now - self.stall_t < STALL:
            self.stall_t = None
        if self.stall_t is not None and STALL < self.now - self.stall_t < STALL + STALL_FOR:
            side = 1.0 if (b[1] < 0) == (u[0] > 0) else -1.0  # desvia a bola para o centro do campo
            a = math.atan2(b[1] - me[1], b[0] - me[0]) + side * math.radians(STALL_ANG)
            return drive(me, me[0] + math.cos(a), me[1] + math.sin(a))
        if self.stall_t is not None and self.now - self.stall_t >= STALL + STALL_FOR:
            self.stall_t = None
        if along < -0.02 and abs(lat) < ALIGN + 0.15 * max(0.0, -along - TOUCH) and abs(head) < ALIGN_ANG + 0.3:
            # alinhado atras da bola: conduz, mirando um ponto adiante na linha bola-alvo
            return drive(me, b[0] + 0.15 * u[0], b[1] + 0.15 * u[1])
        target = (b[0] - BEHIND * u[0], b[1] - BEHIND * u[1])
        if along > -0.04:  # do lado errado (ou ao lado): contorna pelo lado em que estamos
            s = 1.0 if lat >= 0 else -1.0
            n = (-u[1] * s, u[0] * s)
            target = (b[0] + AROUND * n[0] - 0.04 * u[0], b[1] + AROUND * n[1] - 0.04 * u[1])
            if not (-Y_LIM < target[1] < Y_LIM):  # sem espaco junto a parede: pelo outro lado
                target = (b[0] - AROUND * n[0] - 0.04 * u[0], b[1] - AROUND * n[1] - 0.04 * u[1])
        return drive(me, *inside(target))

    # --- apoio --------------------------------------------------------------------

    def support(self, me, st):
        if self.lead >= WALL_LEAD:
            return self.second_keeper(me)
        b = self.ball
        pb, pu = self.push
        ax, ay = me[0] - pb[0], me[1] - pb[1]
        along, lat = ax * pu[0] + ay * pu[1], -ax * pu[1] + ay * pu[0]
        if 0.0 < along < CORRIDOR + 0.1 and abs(lat) < 0.1:
            # estamos no corredor do atacante: sai de lado (senao ele empurra a bola contra nos)
            s = 1.0 if lat >= 0 else -1.0
            if not -Y_LIM < me[1] + s * 0.15 * pu[0] < Y_LIM:
                s = -s
            return drive(me, *inside((me[0] - s * 0.15 * pu[1], me[1] + s * 0.15 * pu[0])))
        if SUPPORT == "trem" and dist(st, b) < TOUCH + 0.03:
            # empurra as costas do atacante na direcao em que ele conduz: 2 contra 1 no contato
            u = unit(b[0] - st[0], b[1] - st[1])
            return drive(me, st[0] - (TRAIN_GAP - 0.03) * u[0], st[1] - (TRAIN_GAP - 0.03) * u[1])
        if SUPPORT == "bloqueio":
            k = min(self.opps, key=lambda k: dist(self.P[k], b))
            o = self.P[k]
            if dist(o, b) < 0.4 and o[0] > -0.8:
                u = unit(b[0] - o[0], b[1] - o[1])
                return drive(me, o[0] + 0.06 * u[0], o[1] + 0.06 * u[1])
        if SUPPORT == "sequestro":
            k = max(self.opps, key=lambda k: self.P[k][0])
            gk = self.P[k]
            if gk[0] > HALF_L - 0.2 and abs(gk[1]) < 0.35:
                s = 1.0 if gk[1] >= 0 else -1.0  # empurra para o lado mais proximo, longe da boca
                p = (gk[0] - 0.01, gk[1] - s * (2 * R - 0.01))
                return drive(me, *inside(p))
        # zaga: entre a bola e o nosso gol, atras do atacante
        x = min(b[0] - 0.2, ZAGA_X if b[0] < 0.2 else 0.0)
        x = max(x, -HALF_L + 0.2)
        y = b[1] * (x + HALF_L + 0.1) / max(b[0] + HALF_L + 0.1, 0.05)
        return arrive(me, *inside((x, y)))

    def second_keeper(self, me):
        y = 0.09 if self.ball[1] >= 0 else -0.09
        return self.keep(me, x0=GK_X + 0.05, y_off=y)

    # --- goleiro ------------------------------------------------------------------

    def keep(self, me, x0=GK_X, y_off=0.0):
        if KEEPER != "frente" and y_off == 0.0:
            return self.keep_side(me)
        b, (vx, vy) = self.ball, self.bv
        if b[0] > LIBERO_X:
            x0 = LIBERO_TO
        if dist(me, b) < CLEAR_R and b[0] > me[0] - 0.01 and b[0] < -0.45:
            # bola encostada na frente: empurra para fora, de lado (nunca parado: parado e empurrado)
            s = 1.0 if b[1] >= me[1] else -1.0
            u = unit(b[0] - me[0] + 0.03, b[1] - me[1] + 0.02 * s)
            return drive(me, b[0] + 0.1 * u[0], b[1] + 0.1 * u[1])
        if vx < -0.03 and b[0] > x0:  # bola vindo: onde ela cruza a nossa linha
            y = b[1] + vy * (x0 - b[0]) / vx
        else:  # parada ou indo embora: na linha do gol ao centro da bola
            y = b[1] * (x0 + HALF_L + 0.12) / max(b[0] + HALF_L + 0.12, 0.05)
        y = clamp(y + y_off, -GK_Y, GK_Y)
        if b[0] < me[0] - 0.01 and abs(b[1]) < 0.3:  # bola atras de nos: tira de lado
            y = b[1] + (0.07 if b[1] < me[1] else -0.07)
        return arrive(me, x0, y)


    def keep_side(self, me):
        """Goleiro do Lucas: de pe na linha (orientado em y), corre de lado sem girar e preve a
        bola com reflexao nas laterais. Com KEEPER = "lado_afasta", tambem afasta a bola que encosta."""
        b, (vx, vy) = self.ball, self.bv
        if KEEPER == "lado_afasta" and dist(me, b) < CLEAR_R and b[0] > me[0] - 0.01 and b[0] < -0.45:
            s = 1.0 if b[1] >= me[1] else -1.0
            u = unit(b[0] - me[0] + 0.03, b[1] - me[1] + 0.02 * s)
            return drive(me, b[0] + 0.1 * u[0], b[1] + 0.1 * u[1])
        y = b[1]
        if vx < -0.02:
            t = (SIDE_X - b[0]) / vx
            if 0 < t < 3:
                y = b[1] + vy * t
                while abs(y) > HALF_W - RB:
                    y = math.copysign(2 * (HALF_W - RB), y) - y
        y = clamp(y, -0.158, 0.158)
        if abs(me[0] - SIDE_X) > 0.018:
            return arrive(me, SIDE_X, y)
        a = wrap(math.pi / 2 - me[2])
        fwd = 1.0
        if abs(a) > math.pi / 2:  # de re tambem serve: o mais perto de +-90 graus
            a, fwd = wrap(a + math.pi), -1.0
        if abs(a) > 0.25:
            return 0.0, clamp(7 * a, -W_MAX, W_MAX)
        return fwd * clamp(9 * (y - me[1]), -V_MAX, V_MAX), clamp(7 * a, -W_MAX, W_MAX)


def team_frame(s, pose, ball, score):
    """Mundo -> nosso gol em x < 0 (s = -1 espelha x para o time vermelho)."""
    return {
        "pose": {k: (s * x, y, yaw if s > 0 else wrap(math.pi - yaw), v) for k, (x, y, yaw, v) in pose.items()},
        "ball": (s * ball[0], ball[1], s * ball[2], ball[3]),
        "score": score,
    }


TEAMS = {"blue": (0, +1), "red": (3, -1)}


class Node:
    def __init__(self, brain_cls=Brain):
        rospy.init_node("soccer")
        team = rospy.get_param("~team", "blue")
        first, self.s = TEAMS[team]
        ids = [first + k for k in range(3)]
        opps = [k for k in range(6) if k not in ids]
        self.mine = 0 if team == "blue" else 1
        self.brain = brain_cls(ids, opps)
        self.ids = ids
        self.pose, self.ball, self.score = {}, None, (0, 0)
        self.pubs = {i: rospy.Publisher("/hero_%d/cmd_vel" % i, Twist, queue_size=1) for i in ids}
        for k in range(6):
            rospy.Subscriber("/hero_%d/odom" % k, Odometry, self.on_odom, callback_args=k, queue_size=1)
        rospy.Subscriber("/ball/odom", Odometry, self.on_ball, queue_size=1)
        rospy.Subscriber("/game/state", String, self.on_game, queue_size=1)

    def on_odom(self, msg, k):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        x, y = p.x, p.y
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose[k] = (x, y, yaw, msg.twist.twist.linear.x)

    def on_ball(self, msg):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        self.ball = (p.x, p.y, v.x, v.y)

    def on_game(self, msg):
        g = json.loads(msg.data)
        sc = (g.get("blue", 0), g.get("red", 0))
        self.score = sc if self.mine == 0 else (sc[1], sc[0])

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.ball is not None and len(self.pose) == 6:
                w = team_frame(self.s, self.pose, self.ball, self.score)
                for i, (v, om) in zip(self.ids, self.brain.step(rospy.get_time(), w)):
                    cmd = Twist()
                    cmd.linear.x, cmd.angular.z = v, self.s * om
                    self.pubs[i].publish(cmd)
            rate.sleep()


if __name__ == "__main__":
    Node().run()
