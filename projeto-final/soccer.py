#!/usr/bin/env python3
"""Futebol HeRo: proteção da linha do gol, disputa e desmarcação autônoma.
Controlador independente; ~team=blue ou red. Ataque em +x no referencial local.

v1 (robusta a ruido): odometria dos seis robos filtrada por EKF (modelo uniciclo, ruido
estimado on-line; nos nossos robos o comando publicado entra no modelo) e bola filtrada por
um Kalman de velocidade constante. A decisao usa o estado previsto LEAD s a frente.
"""
import collections
import math
import threading
import time  # relogio monotonico: o de parede (rospy.get_time) pode saltar (ajuste de horario/NTP; no WSL salta 7 s)

import numpy as np
V_MAX,W_MAX,STALE=.2,4.,.5
K_W=6.
KEEP_X=-.89
LEAD=.05  # s de atraso compensados pela predicao dos filtros
MARK=True
DYNAMIC=False

def clamp(x,a,b):return max(a,min(b,x))
def wrap(a):return math.atan2(math.sin(a),math.cos(a))
wrap_angle=wrap
def distance(a,b):return math.hypot(a[0]-b[0],a[1]-b[1])

def drive(me,target,stop=.01,reverse=True,push=False):
    dx,dy=target[0]-me[0],target[1]-me[1];d=math.hypot(dx,dy)
    if d<stop:return 0.,0.
    a=wrap(math.atan2(dy,dx)-me[2]);sgn=1.
    if reverse and abs(a)>math.pi/2:sgn=-1.;a=wrap(a+math.pi)
    v=.2 if push else min(.2,2.5*d)
    return (sgn*v*max(0.,math.cos(a)) if abs(a)<math.radians(55) else 0.),clamp(K_W*a,-4.,4.)

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


Q_BALL = 0.25   # m/s^1.5: aceleracao nao modelada da bola (chutes, paredes)


class BallTrack:
    """Kalman linear da bola, estado [x, y, vx, vy] (velocidade constante). Mede posicao e a
    velocidade do twist (referencial do mundo). Inovacao grande na velocidade = chute: abre
    a covariancia da velocidade em vez de rejeitar a medida."""

    def __init__(self):
        self.noise = Noise()
        self.s = None
        self.P = None
        self.t = 0.0
        self.raw = collections.deque(maxlen=3)
        self.bad = 0

    def _reset(self, t, z):
        sp = self.noise.sigma["p"]
        self.s = np.array(z, dtype=float)
        self.P = np.diag([sp * sp, sp * sp, 0.05 ** 2, 0.05 ** 2])
        self.t, self.bad = t, 0

    @staticmethod
    def _predict(s, P, dt):
        if dt <= 0:
            return s, P
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        q = Q_BALL * Q_BALL
        Q = q * np.array([[dt ** 3 / 3, 0, dt ** 2 / 2, 0], [0, dt ** 3 / 3, 0, dt ** 2 / 2],
                          [dt ** 2 / 2, 0, dt, 0], [0, dt ** 2 / 2, 0, dt]])
        return F @ s, F @ P @ F.T + Q

    def update(self, t, x, y, vx, vy, sig=None):
        """sig: (sx, sy, svx, svy) da covariancia de /ball/odom; sem ela, estimativa on-line."""
        nz = self.noise
        self.raw.append((x, y, vx, vy))
        if len(self.raw) == 3:
            (x0, y0, a0, b0), (x1, y1, a1, b1), _ = self.raw
            nz.add("p", x - 2 * x1 + x0)
            nz.add("p", y - 2 * y1 + y0)
            nz.add("v", vx - 2 * a1 + a0)
            nz.add("v", vy - 2 * b1 + b0)
        if self.s is None:
            self._reset(t, (x, y, vx, vy))
            return
        s, P = self._predict(self.s, self.P, t - self.t)
        fl = Noise.FLOOR
        if sig is not None:
            sx, sy = max(sig[0], fl["p"]), max(sig[1], fl["p"])
            svx, svy = max(sig[2], fl["v"]), max(sig[3], fl["v"])
        else:
            sx = sy = nz.sigma["p"]
            svx = svy = nz.sigma["v"]
        sp, sv = max(sx, sy), max(svx, svy)
        nu = np.array([x - s[0], y - s[1]])
        S = P[:2, :2] + np.diag([sx * sx, sy * sy])
        jump = math.hypot(nu[0], nu[1])
        gate = max(GATE, 6.0 * math.sqrt(S[0, 0] + S[1, 1]))
        if jump > gate:
            self.bad += 1
            if self.bad >= 2 or jump > max(TELEPORT, 2.0 * gate):  # gol: a bola volta ao centro
                self._reset(t, (x, y, vx, vy))
            else:
                self.s, self.P, self.t = s, P, max(t, self.t)
            return
        self.bad = 0
        nv = np.array([vx - s[2], vy - s[3]])
        if math.hypot(nv[0], nv[1]) > 4.0 * math.sqrt(P[2, 2] + P[3, 3] + 2 * sv * sv):
            P[2:, 2:] += np.eye(2) * float(nv @ nv)  # chute: confia na medida nova
        z = np.array([x, y, vx, vy])
        S = P + np.diag([sx * sx, sy * sy, svx * svx, svy * svy])
        K = P @ np.linalg.inv(S)
        s = s + K @ (z - s)
        P = P - K @ P
        self.s, self.P, self.t = s, 0.5 * (P + P.T), max(t, self.t)

    def get(self, t):
        if self.s is None:
            return None
        s, _ = self._predict(self.s, self.P, t - self.t)
        return tuple(float(v) for v in s)
# === fim do estimador ======================================================================


class Brain:
    def __init__(self,team='blue'):
        if team not in ('blue','red'):raise ValueError(team)
        self.first,self.sign=(0,1) if team=='blue' else (3,-1)
        self.striker,self.changed=1,-1e9
        self.anchor,self.progress=None,0.
        self.recover_until=[0.]*3;self.last=[None]*3;self.stuck=[0.]*3
        self.states=['goleiro','ataque','apoio'];self.now=0.
    def frame(self,poses,ball):
        s=self.sign;ids=list(range(self.first,self.first+3));ids += [i for i in range(6) if i not in ids]
        return [(s*poses[i][0],s*poses[i][1],wrap(poses[i][2]+(math.pi if s<0 else 0))) for i in ids],tuple(s*v for v in ball)
    @staticmethod
    def bounded(p):
        x,y=p
        # O bolsão do gol é acessível. Fora da abertura, respeitar as paredes.
        lim=.96 if abs(y)<.15 else .855
        return clamp(x,-lim,lim),clamp(y,-.557,.557)
    def goalkeeper(self,p,b):
        me=p[0];bx,by,vx,vy=b
        y=by
        if vx<-.02:
            t=(KEEP_X-bx)/vx
            if 0<t<3:
                y=by+vy*t
                # Reflexão geométrica da bola nas laterais (raio 2,14 cm).
                while y>.579 or y<-.579:
                    y=1.158-y if y>.579 else -1.158-y
        y=clamp(y,-.158,.158)
        tx=KEEP_X
        # Nunca sai da área para atacar. A orientação vertical permite ir/voltar
        # ao longo do gol sem gastar meio segundo girando a cada correção de y.
        if abs(me[0]-tx)>.018:
            return drive(me,(tx,y),stop=.006)
        a=wrap(math.pi/2-me[2])
        if abs(a)>.25:return 0.,clamp(7*a,-4,4)
        return clamp(9*(y-me[1]),-.2,.2),clamp(7*a,-4,4)
    def attack(self,i,p,b):
        me=p[i];bx,by=b[:2]
        gx,gy=.97,0.
        dx,dy=gx-bx,gy-by;n=max(math.hypot(dx,dy),1e-6);ux,uy=dx/n,dy/n
        rx,ry=me[0]-bx,me[1]-by
        behind=-(rx*ux+ry*uy);lat=-rx*uy+ry*ux
        target=(bx-.10*ux,by-.10*uy)
        self.states[i]='aproximacao'
        if behind>.025 and abs(lat)<.04:
            target=(bx+.10*ux,by+.10*uy);self.states[i]='chute'
        target=self.bounded(target)
        ex,ey=target[0]-me[0],target[1]-me[1]
        a=wrap(math.atan2(ey,ex)-me[2])
        return min(.2,2.5*math.hypot(ex,ey))*max(0.,math.cos(a)),clamp(8*a,-4,4)
    def support(self,i,p,b):
        me=p[i]
        if MARK:
            # Marca por contato o jogador de campo mais próximo da bola.
            # A força do apoio age no adversário, liberando o atacante.
            q=min(p[4:6],key=lambda q:distance(q,b))
            self.states[i]='marcacao'
            return drive(me,q,stop=.002,reverse=False,push=True)
        side=-.23 if b[1]>=0 else .23
        target=self.bounded((clamp(b[0]-.23,-.62,.58),clamp(b[1]+side,-.43,.43)))
        return drive(me,target,reverse=True)
    def step(self,now,poses,ball):
        self.now=now;p,b=self.frame(poses,ball)
        if self.anchor is None or distance(b,self.anchor)>.035:self.anchor=b[:2];self.progress=now
        costs={i:distance(p[i],b)+.6*max(0,p[i][0]-b[0]) for i in (1,2)}
        other=3-self.striker
        if DYNAMIC and now-self.changed>1. and costs[other]+.10<costs[self.striker]:self.striker=other;self.changed=now
        self.states=['goleiro','apoio','apoio'];out=[self.goalkeeper(p,b),(0.,0.),(0.,0.)]
        out[self.striker]=self.attack(self.striker,p,b)
        helper=3-self.striker;out[helper]=self.support(helper,p,b)
        # Se o atrito impede o giro ou a bola fica presa, recua para soltar o
        # contato antes de reorientar. O apoio não empurra o robô que recua.
        for i in (1,2):
            if self.last[i] is None or distance(p[i],self.last[i])>.025:self.last[i]=p[i][:2];self.stuck[i]=now
            if now-self.stuck[i]>2.2 and abs(out[i][0])>.025:
                self.recover_until[i]=now+.65;self.stuck[i]=now;self.last[i]=p[i][:2]
            if now<self.recover_until[i]:
                self.states[i]='liberar';out[i]=(-.16,0.)
        return out



class Core:
    """Time sem ROS: on_odom/on_ball recebem as medidas (com o instante de chegada) e
    control(t) devolve {robo: (v, w)}; para os robos se alguma medida estiver velha."""

    def __init__(self, team):
        self.brain = Brain(team)
        self.ids = list(range(self.brain.first, self.brain.first + 3))
        self.noise = Noise()
        self.tracks = {i: Track(self.noise, own=(i in self.ids)) for i in range(6)}
        self.ball = BallTrack()
        self.last = {}
        self.lock = threading.Lock()

    def on_odom(self, i, t, x, y, yaw, v, w, sig=None):
        if i in self.tracks:
            with self.lock:
                self.tracks[i].update(t, x, y, yaw, v, w, sig)
                self.last[i] = t

    def on_ball(self, t, x, y, vx, vy, sig=None):
        with self.lock:
            self.ball.update(t, x, y, vx, vy, sig)
            self.last["ball"] = t

    def control(self, t):
        with self.lock:
            if len(self.last) < 7 or t - min(self.last.values()) > STALE:
                return {i: (0.0, 0.0) for i in self.ids}
            poses = {i: tr.get(t + LEAD)[:3] for i, tr in self.tracks.items()}
            ball = self.ball.get(t + LEAD)
        out = self.brain.step(t, poses, ball)
        cmds = {}
        with self.lock:
            for i, (v, w) in zip(self.ids, out):
                v, w = clamp(v, -V_MAX, V_MAX), clamp(w, -W_MAX, W_MAX)
                self.tracks[i].set_cmd(v, w)
                cmds[i] = (v, w)
        return cmds


class Soccer:
    def __init__(self):
        import rospy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        self.ros, self.Twist = rospy, Twist
        rospy.init_node("soccer")
        self.core = Core(rospy.get_param("~team", "blue"))
        self.ids = self.core.ids
        self.pubs = {i: rospy.Publisher("/hero_%d/cmd_vel" % i, Twist, queue_size=1) for i in self.ids}
        self.subs = [rospy.Subscriber("/hero_%d/odom" % i, Odometry, self.on_odom,
                                     callback_args=i, queue_size=1) for i in range(6)]
        self.subs.append(rospy.Subscriber("/ball/odom", Odometry, self.on_ball, queue_size=1))
        rospy.on_shutdown(self.stop)

    def on_odom(self, msg, i):
        q, p = msg.pose.pose.orientation, msg.pose.pose.position
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        tw = msg.twist.twist
        self.core.on_odom(i, time.monotonic(), p.x, p.y, yaw, tw.linear.x, tw.angular.z, sigmas_from_odom(msg))

    def on_ball(self, msg):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        sig = None
        try:
            pc, tc = msg.pose.covariance, msg.twist.covariance
            if len(pc) >= 36 and pc[0] > 0.0:
                sig = (math.sqrt(pc[0]), math.sqrt(pc[7]), math.sqrt(max(tc[0], 0.0)), math.sqrt(max(tc[7], 0.0)))
        except (AttributeError, TypeError, ValueError):
            pass
        self.core.on_ball(time.monotonic(), p.x, p.y, v.x, v.y, sig)

    def publish(self, commands):
        for i, (v, w) in commands.items():
            cmd = self.Twist()
            cmd.linear.x, cmd.angular.z = clamp(v, -V_MAX, V_MAX), clamp(w, -W_MAX, W_MAX)
            self.pubs[i].publish(cmd)

    def stop(self):
        self.publish({i: (0.0, 0.0) for i in self.ids})

    def run(self):
        rate = self.ros.Rate(20)
        while not self.ros.is_shutdown():
            cmds = self.core.control(time.monotonic())
            if len(self.core.last) < 7:
                self.ros.logwarn_throttle(2, "aguardando odometria dos seis robos e da bola")
            self.publish(cmds)
            rate.sleep()


if __name__ == "__main__":
    Soccer().run()
