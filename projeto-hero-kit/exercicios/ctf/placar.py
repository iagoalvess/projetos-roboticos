#!/usr/bin/env python3
"""Registra uma partida de rouba-bandeira: eventos (pontos, seguradas, coletas) e o placar final.

Uso, dentro do container ROS:  python3 placar.py DURACAO_S nome semente
Conta o tempo do simulador a partir do primeiro comando de qualquer robo. Grava uma linha por
evento em ctf/log_ctf_<nome>_<data>.csv e imprime o resumo.
"""
import csv
import json
import math
import os
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from psr_hero.utils import pose_from_odom
from std_msgs.msg import String

DUR, NAME, SEED = float(sys.argv[1]), sys.argv[2], sys.argv[3]
st = {"t0": None, "prev": None, "rows": [], "last": None, "moved": False, "tp": -1.0}
pose = {}


def on_odom(msg, i):
    pose[i] = pose_from_odom(msg)


def on_cmd(msg):
    st["moved"] = True


def on_game(msg):
    g = json.loads(msg.data)
    if st["t0"] is None:
        if not st["moved"]:
            return
        st["t0"], st["s0"] = g["t"], list(g["score"])
    t = round(g["t"] - st["t0"], 2)
    p = st["prev"]
    if p is not None:
        for team in (0, 1):
            if g["score"][team] > p["score"][team]:
                st["rows"].append([t, "ponto", "azul" if team == 0 else "vermelho", ""])
        for i, (h, c) in enumerate(zip(g["held"], g["carrying"])):
            if h and not p["held"][i]:
                st["rows"].append([t, "segurado", "hero_%d" % i, ""])
            if c and not p["carrying"][i]:
                st["rows"].append([t, "coleta", "hero_%d" % i, ""])
        if p["round_time"] > 85.0 and g["round_time"] < p["round_time"] - 1.0:  # 90 s sem ponto
            st["rows"].append([t, "reinicio", "", ""])
    if t - st["tp"] >= 0.5 and len(pose) == 4:  # trilha a 2 Hz
        st["tp"] = t
        st["rows"].append([t, "pose", "carrega " + "".join(str(i) for i, c in enumerate(g["carrying"]) if c),
                           " ".join("%.3f %.3f %.0f" % (pose[i][0], pose[i][1], math.degrees(pose[i][2])) for i in range(4))])
    st["prev"], st["last"] = g, t


rospy.init_node("placar_ctf")
for i in range(4):
    rospy.Subscriber("/hero_%d/cmd_vel" % i, Twist, on_cmd, queue_size=1)
    rospy.Subscriber("/hero_%d/odom" % i, Odometry, on_odom, callback_args=i, queue_size=1)
rospy.Subscriber("/game/state", String, on_game, queue_size=1)
while not rospy.is_shutdown() and (st["last"] is None or st["last"] < DUR):
    rospy.sleep(0.5)

g = st["prev"]
score = [g["score"][k] - st["s0"][k] for k in (0, 1)]
path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "log_ctf_%s_%s.csv" % (NAME, time.strftime("%Y%m%d_%H%M%S")))
with open(path, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t_s", "evento", "quem", "obs"])
    w.writerows(st["rows"])
    w.writerow([st["last"], "fim", "azul %d x %d vermelho" % tuple(score), "semente %s" % SEED])
ev = [r[1] + ":" + r[2] for r in st["rows"]]
print("FIM %s (semente %s): azul %d x %d vermelho | coletas azul %d vermelho %d | seguradas hero_0..3 %s -> %s" % (
    NAME, SEED, score[0], score[1],
    sum(1 for r in st["rows"] if r[1] == "coleta" and r[2] in ("hero_0", "hero_1")),
    sum(1 for r in st["rows"] if r[1] == "coleta" and r[2] in ("hero_2", "hero_3")),
    [ev.count("segurado:hero_%d" % i) for i in range(4)], os.path.basename(path)), flush=True)
