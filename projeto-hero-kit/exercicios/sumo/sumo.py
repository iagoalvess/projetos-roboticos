#!/usr/bin/env python3
"""Sumo: empurra sempre a V_MAX, de frente para o adversario.

No simulador, quem comanda menos velocidade no contato perde o empurrao
(ver teste_fisica.py). Quando a briga trava, o robo empurra com o rumo
desviado para arrastar os dois para a borda do adversario.
"""
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom, wrap_angle

RING_RADIUS = 0.40
R_HARD = 0.35       # m, sempre volta para o centro
R_GUARD = 0.28      # m, volta se estiver solto e mais perto da borda que o adversario
K_W = 5.0           # rad/s por rad
TURN_FIRST = math.radians(60)
CONTACT = 0.085     # m entre centros
STALL_SPEED = 0.03  # m/s
STALL_TIME = 1.0    # s
DRAG = math.radians(20)


def head_to(me, tx, ty, offset=0.0):
    x, y, yaw = me
    a = wrap_angle(math.atan2(ty - y, tx - x) + offset - yaw)
    v = V_MAX if abs(a) < TURN_FIRST else 0.0
    return v, clamp(K_W * a, -W_MAX, W_MAX)


def back_to_center(me):
    x, y, yaw = me
    a = wrap_angle(math.atan2(-y, -x) - yaw)
    if abs(a) > math.pi / 2:  # centro atras: volta de re
        a = wrap_angle(a - math.pi)
        return -V_MAX, clamp(K_W * a, -W_MAX, W_MAX)
    return V_MAX, clamp(K_W * a, -W_MAX, W_MAX)


class Sumo:
    def __init__(self):
        rospy.init_node("sumo")
        self.robot = rospy.get_param("~robot", 0)
        self.opponent = rospy.get_param("~opponent", 1)
        self.me = None
        self.speed = 0.0
        self.opp = None
        self.opp_speed = 0.0
        self.stall_since = None
        self.drag_side = 0.0
        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % self.robot, Twist, queue_size=1)
        rospy.Subscriber("/hero_%d/odom" % self.robot, Odometry, self.on_me, queue_size=1)
        rospy.Subscriber("/hero_%d/odom" % self.opponent, Odometry, self.on_opp, queue_size=1)

    def on_me(self, msg):
        self.me = pose_from_odom(msg)
        self.speed = msg.twist.twist.linear.x

    def on_opp(self, msg):
        self.opp = pose_from_odom(msg)
        self.opp_speed = msg.twist.twist.linear.x

    def compute_command(self, me, opp, speed, now):
        x, y, _ = me
        ox, oy, _ = opp
        r_me, r_opp = math.hypot(x, y), math.hypot(ox, oy)
        dx, dy = ox - x, oy - y
        d = max(math.hypot(dx, dy), 1e-6)

        if r_me > R_HARD or (d > CONTACT + 0.02 and r_me > R_GUARD and r_me > r_opp - 0.02):
            self.stall_since, self.drag_side = None, 0.0
            return back_to_center(me)

        # briga travada: o lado do arrasto e escolhido uma vez por contato
        if d > CONTACT + 0.015 or abs(speed) > 2 * STALL_SPEED:
            self.stall_since, self.drag_side = None, 0.0
        elif self.drag_side == 0.0:
            if abs(speed) < STALL_SPEED and abs(self.opp_speed) < STALL_SPEED:
                if self.stall_since is None:
                    self.stall_since = now
                elif now - self.stall_since > STALL_TIME:
                    self.drag_side = 1.0 if (-dy * ox + dx * oy) >= 0 else -1.0
            else:
                self.stall_since = None
        if self.drag_side != 0.0:
            return head_to(me, ox, oy, self.drag_side * DRAG)

        return head_to(me, ox, oy)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.me is not None and self.opp is not None:
                v, w = self.compute_command(self.me, self.opp, self.speed, rospy.get_time())
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = v, w
                self.pub.publish(cmd)
            rate.sleep()


if __name__ == "__main__":
    Sumo().run()
