#!/usr/bin/env python3
"""Oponente A (versao anterior): a lei do goto do TP5 apontada para o adversario.

v = K_RHO * rho * cos(alpha), w = K_ALPHA * alpha, com a meta no centro do
adversario; volta para o centro do ringue quando passa de R_GUARD.
"""
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom, wrap_angle

RING_RADIUS = 0.40
R_GUARD = 0.30  # m
K_RHO = 1.5     # 1/s
K_ALPHA = 3.0   # 1/s


def goto(me, tx, ty):
    x, y, yaw = me
    rho = math.hypot(tx - x, ty - y)
    a = wrap_angle(math.atan2(ty - y, tx - x) - yaw)
    v = K_RHO * rho * max(0.0, math.cos(a))
    return clamp(v, 0.0, V_MAX), clamp(K_ALPHA * a, -W_MAX, W_MAX)


class Sumo:
    def __init__(self):
        rospy.init_node("sumo")
        self.robot = rospy.get_param("~robot", 0)
        self.opponent = rospy.get_param("~opponent", 1)
        self.me = None
        self.speed = 0.0
        self.opp = None
        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % self.robot, Twist, queue_size=1)
        rospy.Subscriber("/hero_%d/odom" % self.robot, Odometry, self.on_me, queue_size=1)
        rospy.Subscriber("/hero_%d/odom" % self.opponent, Odometry, self.on_opp, queue_size=1)

    def on_me(self, msg):
        self.me = pose_from_odom(msg)
        self.speed = msg.twist.twist.linear.x

    def on_opp(self, msg):
        self.opp = pose_from_odom(msg)

    def compute_command(self, me, opp, speed, now):
        if math.hypot(me[0], me[1]) > R_GUARD:
            return goto(me, 0.0, 0.0)
        return goto(me, opp[0], opp[1])

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
