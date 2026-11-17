#!/usr/bin/env python3
"""Registra o resultado de cada rodada do sumo em CSV e para depois de N rodadas.

Uso, dentro do container ROS:  python3 placar.py N nome
Grava sumo/log_sumo_<nome>_<data>.csv com: rodada, resultado, duracao (s),
e a pose dos dois robos no fim da rodada.
"""
import csv
import json
import os
import sys
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from psr_hero.utils import pose_from_odom

N = int(sys.argv[1])
NAME = sys.argv[2]
pose = [None, None]
state = {"prev": None, "t": 0.0, "last_pose": None, "rows": []}


def on_odom(msg, i):
    pose[i] = pose_from_odom(msg)


def on_game(msg):
    g = json.loads(msg.data)
    key = (tuple(g["wins"]), g["draws"])
    if state["prev"] is None:
        state["prev"] = key
    if key != state["prev"]:
        (w0, w1), d = state["prev"]
        res = "hero_0" if g["wins"][0] > w0 else "hero_1" if g["wins"][1] > w1 else "empate"
        p = state["last_pose"] or [(0, 0, 0), (0, 0, 0)]
        state["rows"].append([len(state["rows"]) + 1, res, round(state["t"], 2),
                              *[round(c, 3) for c in p[0][:2]], *[round(c, 3) for c in p[1][:2]]])
        print("rodada %d: %s em %.1f s" % (len(state["rows"]), res, state["t"]), flush=True)
        state["prev"] = key
    elif g["round_time"] > 0.2:  # durante a queda o tempo continua, a pose congela no fim
        state["t"] = g["round_time"]
        if None not in pose:
            state["last_pose"] = [tuple(p) for p in pose]


rospy.init_node("placar_sumo")
for i in (0, 1):
    rospy.Subscriber("/hero_%d/odom" % i, Odometry, on_odom, callback_args=i, queue_size=1)
rospy.Subscriber("/game/state", String, on_game, queue_size=1)
while not rospy.is_shutdown() and len(state["rows"]) < N:
    rospy.sleep(0.2)

path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "log_sumo_%s_%s.csv" % (NAME, time.strftime("%Y%m%d_%H%M%S")))
with open(path, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["rodada", "vencedor", "duracao_s", "x0", "y0", "x1", "y1"])
    w.writerows(state["rows"][:N])
res = [r[1] for r in state["rows"][:N]]
print("FIM %s: hero_0=%d hero_1=%d empates=%d -> %s" % (
    NAME, res.count("hero_0"), res.count("hero_1"), res.count("empate"), os.path.basename(path)), flush=True)
