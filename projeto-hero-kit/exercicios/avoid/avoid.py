#!/usr/bin/env python3
"""Desvio de obstaculos: controle so com o lidar; odometria so para o registro."""
import csv
import json
import os
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom


class Avoid:
    LABEL = "A"
    DURATION = 300.0   # s de tempo simulado
    V_CRUISE = 0.15    # m/s
    V_MIN = 0.04       # m/s, sem esse piso o robo para antes de D_TURN
    D_SLOW = 0.30      # m
    D_TURN = 0.12      # m
    D_CLEAR = 0.20     # m
    K_W = 4.0          # rad/s por m
    W_TURN = 2.0       # rad/s

    def __init__(self):
        rospy.init_node("avoid")
        self.robot = rospy.get_param("~robot", 0)
        self.scan = None
        self.pose = None
        self.t = None
        self.turn_dir = 0  # 0: andando; +1/-1: girando no lugar

        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % self.robot, Twist, queue_size=1)
        rospy.Subscriber("/hero_%d/laser" % self.robot, LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber("/hero_%d/odom" % self.robot, Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber("/game/state", String, self.on_state, queue_size=1)

    def on_scan(self, scan):
        self.scan = scan

    def on_odom(self, msg):
        self.pose = pose_from_odom(msg)

    def on_state(self, msg):
        self.t = json.loads(msg.data)["t"]

    def compute_command(self, ranges):
        front = min(ranges[3], ranges[4], ranges[5])
        left = ranges[5] + ranges[6]
        right = ranges[2] + ranges[3]

        if self.turn_dir != 0 and front > self.D_CLEAR:
            self.turn_dir = 0
        elif self.turn_dir == 0 and front < self.D_TURN:
            self.turn_dir = 1 if left >= right else -1
        if self.turn_dir != 0:
            return 0.0, self.turn_dir * self.W_TURN

        slow = clamp((front - self.D_TURN) / (self.D_SLOW - self.D_TURN), 0.0, 1.0)
        v = self.V_MIN + (self.V_CRUISE - self.V_MIN) * slow
        w = self.K_W * (left - right)
        return clamp(v, 0.0, V_MAX), clamp(w, -W_MAX, W_MAX)

    def publish(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub.publish(cmd)

    def run(self):
        folder = os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(folder, "log_avoid_%s_%s.csv" % (self.LABEL, time.strftime("%Y%m%d_%H%M%S")))
        rate = rospy.Rate(20)
        t0 = None
        with open(path, "w", newline="") as f:
            log = csv.writer(f)
            log.writerow(["t", "x", "y", "yaw", "min_range", "v", "w"])
            while not rospy.is_shutdown():
                if self.scan is None or self.pose is None or self.t is None:
                    rospy.logwarn_throttle(2.0, "aguardando lidar, odometria e /game/state")
                else:
                    if t0 is None:
                        t0 = self.t
                    ranges = list(self.scan.ranges)
                    v, w = self.compute_command(ranges)
                    self.publish(v, w)
                    log.writerow([round(self.t - t0, 3), *self.pose, min(ranges), v, w])
                    if self.t - t0 >= self.DURATION:
                        self.publish(0.0, 0.0)
                        rospy.loginfo("ensaio concluido: %s", path)
                        return
                rate.sleep()


if __name__ == "__main__":
    try:
        Avoid().run()
    except rospy.ROSInterruptException:
        pass
