#!/usr/bin/env python3
"""Testes de empurrao no ringue do sumo (rodar com robot0:=none robot1:=none).

Cada teste dura uma rodada: os dois robos recebem comandos fixos e o script
registra para onde a dupla andou. Uso, dentro do container ROS:
  python3 teste_fisica.py igual fraco parado angulo30 lateral
"""
import json
import math
import sys

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from psr_hero.utils import clamp, pose_from_odom, wrap_angle

pose = [None, None]
game = {}


def on_odom(msg, i):
    pose[i] = pose_from_odom(msg)


def on_game(msg):
    game.update(json.loads(msg.data))


def aim(me, tx, ty, v, offset=0.0):
    """Comando para andar a v apontando para (tx, ty) com um desvio de offset rad."""
    a = wrap_angle(math.atan2(ty - me[1], tx - me[0]) + offset - me[2])
    return v * max(0.0, math.cos(a)), clamp(4.0 * a, -4.0, 4.0)


TESTS = {  # nome: (comando do hero_0, comando do hero_1), cada um f(me, opp)
    "igual": (lambda m, o: aim(m, o[0], o[1], 0.2), lambda m, o: aim(m, o[0], o[1], 0.2)),
    "fraco": (lambda m, o: aim(m, o[0], o[1], 0.2), lambda m, o: aim(m, o[0], o[1], 0.05)),
    "parado": (lambda m, o: aim(m, o[0], o[1], 0.2), lambda m, o: (0.0, 0.0)),
    "angulo30": (lambda m, o: aim(m, o[0], o[1], 0.2, math.radians(30)), lambda m, o: aim(m, o[0], o[1], 0.2)),
    "lateral": (lambda m, o: aim(m, o[0], o[1], 0.2),
                lambda m, o: aim(m, o[0], o[1], 0.2) if game.get("round_time", 0) < 1.0 else (0.2, 0.0)),
}


def main():
    rospy.init_node("teste_fisica")
    pubs = [rospy.Publisher("/hero_%d/cmd_vel" % i, Twist, queue_size=1) for i in (0, 1)]
    for i in (0, 1):
        rospy.Subscriber("/hero_%d/odom" % i, Odometry, on_odom, callback_args=i, queue_size=1)
    rospy.Subscriber("/game/state", String, on_game, queue_size=1)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and (None in pose or "wins" not in game):
        rate.sleep()

    for name in sys.argv[1:]:
        f0, f1 = TESTS[name]
        # espera o inicio de uma rodada nova
        prev = (sum(game["wins"]), game["draws"])
        while not rospy.is_shutdown() and game.get("round_time", 99) > 0.3:
            for p in pubs:
                p.publish(Twist())
            rate.sleep()
        prev = (tuple(game["wins"]), game["draws"])
        start = [p[:2] for p in pose]
        marks = {}
        while not rospy.is_shutdown():
            if (tuple(game["wins"]), game["draws"]) != prev:
                break
            t = game.get("round_time", 0.0)
            for k in (1, 3, 6, 10, 20):
                if t >= k and k not in marks:
                    marks[k] = [(round(p[0], 3), round(p[1], 3)) for p in pose]
            for i, f in enumerate((f0, f1)):
                v, w = f(pose[i], pose[1 - i])
                c = Twist()
                c.linear.x, c.angular.z = v, w
                pubs[i].publish(c)
            rate.sleep()
        wins = tuple(game["wins"])
        result = "empate" if wins == prev[0] else ("hero_0 venceu" if wins[0] > prev[0][0] else "hero_1 venceu")
        print("%-9s inicio=%s %s -> %s (t=%.1f s)" % (
            name, [tuple(round(c, 3) for c in s) for s in start], marks, result, t), flush=True)


if __name__ == "__main__":
    main()
