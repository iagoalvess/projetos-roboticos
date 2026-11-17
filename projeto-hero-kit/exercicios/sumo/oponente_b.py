#!/usr/bin/env python3
"""Oponente B: vai sempre a V_MAX contra o centro do outro.

Volta para o centro do ringue quando passa de R_GUARD.
"""
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom, wrap_angle

RING_RADIUS = 0.40
R_GUARD = 0.30  # m
K_W = 4.0       # rad/s por rad


def drive_to(me, tx, ty):
    """Vai ate (tx, ty) de frente: v reduzida pelo cos do erro de orientacao."""
    x, y, yaw = me
    a = wrap_angle(math.atan2(ty - y, tx - x) - yaw)
    return V_MAX * max(0.0, math.cos(a)), clamp(K_W * a, -W_MAX, W_MAX)


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
            return drive_to(me, 0.0, 0.0)
        return drive_to(me, opp[0], opp[1])

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
