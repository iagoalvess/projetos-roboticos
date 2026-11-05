#!/usr/bin/env python3
"""Navegacao ate metas: proporcional em alpha, v reduzida quando alpha e grande."""
import csv
import json
import math
import os
import time

import rospy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom, wrap_angle


class Goto:
    LABEL = "A"
    DURATION = 180.0  # s de tempo simulado
    K_RHO = 1.5       # 1/s
    K_ALPHA = 3.0     # 1/s

    def __init__(self):
        rospy.init_node("goto")
        self.robot = rospy.get_param("~robot", 0)
        self.pose = None
        self.goal = None
        self.t = None
        self.reached = 0

        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % self.robot, Twist, queue_size=1)
        rospy.Subscriber("/hero_%d/odom" % self.robot, Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber("/hero_%d/goal" % self.robot, PointStamped, self.on_goal, queue_size=1)
        rospy.Subscriber("/game/state", String, self.on_state, queue_size=1)

    def on_odom(self, msg):
        self.pose = pose_from_odom(msg)

    def on_goal(self, msg):
        self.goal = (msg.point.x, msg.point.y)

    def on_state(self, msg):
        state = json.loads(msg.data)
        self.t = state["t"]
        self.reached = state["reached"][self.robot]

    def compute_command(self, pose, goal):
        x, y, yaw = pose
        gx, gy = goal
        rho = math.hypot(gx - x, gy - y)
        alpha = wrap_angle(math.atan2(gy - y, gx - x) - yaw)

        w = self.K_ALPHA * alpha
        v = self.K_RHO * rho * max(0.0, math.cos(alpha))
        return clamp(v, 0.0, V_MAX), clamp(w, -W_MAX, W_MAX)

    def publish(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub.publish(cmd)

    def run(self):
        folder = os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(folder, "log_goto_%s_%s.csv" % (self.LABEL, time.strftime("%Y%m%d_%H%M%S")))
        rate = rospy.Rate(20)
        t0 = None
        reached0 = 0
        with open(path, "w", newline="") as f:
            log = csv.writer(f)
            log.writerow(["t", "x", "y", "yaw", "gx", "gy", "v", "w", "reached"])
            while not rospy.is_shutdown():
                if self.pose is None or self.goal is None or self.t is None:
                    rospy.logwarn_throttle(2.0, "aguardando odometria, meta e /game/state")
                else:
                    if t0 is None:
                        t0, reached0 = self.t, self.reached
                    v, w = self.compute_command(self.pose, self.goal)
                    self.publish(v, w)
                    log.writerow([round(self.t - t0, 3), *self.pose, *self.goal, v, w, self.reached - reached0])
                    if self.t - t0 >= self.DURATION:
                        self.publish(0.0, 0.0)
                        rospy.loginfo("ensaio concluido: %s", path)
                        return
                rate.sleep()


if __name__ == "__main__":
    try:
        Goto().run()
    except rospy.ROSInterruptException:
        pass
