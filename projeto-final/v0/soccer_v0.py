#!/usr/bin/env python3
"""Futebol HeRo: proteção da linha do gol, disputa e desmarcação autônoma.
Controlador independente; ~team=blue ou red. Ataque em +x no referencial local.
"""
import math
import threading
import time
V_MAX,W_MAX,STALE=.2,4.,.5
K_W=6.
KEEP_X=-.89
MARK=True
DYNAMIC=False

def clamp(x,a,b):return max(a,min(b,x))
def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def distance(a,b):return math.hypot(a[0]-b[0],a[1]-b[1])

def drive(me,target,stop=.01,reverse=True,push=False):
    dx,dy=target[0]-me[0],target[1]-me[1];d=math.hypot(dx,dy)
    if d<stop:return 0.,0.
    a=wrap(math.atan2(dy,dx)-me[2]);sgn=1.
    if reverse and abs(a)>math.pi/2:sgn=-1.;a=wrap(a+math.pi)
    v=.2 if push else min(.2,2.5*d)
    return (sgn*v*max(0.,math.cos(a)) if abs(a)<math.radians(55) else 0.),clamp(K_W*a,-4.,4.)

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



class Soccer:
    def __init__(self):
        import rospy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        self.ros, self.Twist = rospy, Twist
        rospy.init_node("soccer")
        self.brain = Brain(rospy.get_param("~team", "blue"))
        self.ids = list(range(self.brain.first, self.brain.first + 3))
        self.lock = threading.Lock()
        self.poses, self.received, self.ball, self.ball_received = {}, {}, None, 0.0
        self.pubs = {i: rospy.Publisher("/hero_%d/cmd_vel" % i, Twist, queue_size=1) for i in self.ids}
        self.subs = [rospy.Subscriber("/hero_%d/odom" % i, Odometry, self.on_odom,
                                     callback_args=i, queue_size=1) for i in range(6)]
        self.subs.append(rospy.Subscriber("/ball/odom", Odometry, self.on_ball, queue_size=1))
        rospy.on_shutdown(self.stop)

    def on_odom(self, msg, i):
        q, p = msg.pose.pose.orientation, msg.pose.pose.position
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.poses[i] = (p.x, p.y, yaw)
            self.received[i] = time.monotonic()

    def on_ball(self, msg):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        with self.lock:
            self.ball, self.ball_received = (p.x, p.y, v.x, v.y), time.monotonic()

    def publish(self, commands):
        for i, (v, w) in zip(self.ids, commands):
            cmd = self.Twist()
            cmd.linear.x, cmd.angular.z = clamp(v, -V_MAX, V_MAX), clamp(w, -W_MAX, W_MAX)
            self.pubs[i].publish(cmd)

    def stop(self):
        self.publish([(0.0, 0.0)] * 3)

    def run(self):
        rate = self.ros.Rate(20)
        while not self.ros.is_shutdown():
            with self.lock:
                poses, ball = dict(self.poses), self.ball
                fresh = len(poses) == 6 and ball is not None and time.monotonic() - min(
                    list(self.received.values()) + [self.ball_received]) < STALE
            if fresh:
                self.publish(self.brain.step(self.ros.get_time(), poses, ball))
            else:
                self.stop()
                self.ros.logwarn_throttle(2, "aguardando odometria atualizada dos seis robos e da bola")
            rate.sleep()


if __name__ == "__main__":
    Soccer().run()
