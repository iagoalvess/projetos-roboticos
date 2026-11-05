#!/usr/bin/env python3
"""Segunda configuracao: mais rapida e mais perto dos obstaculos."""
import rospy

from avoid import Avoid


class AvoidB(Avoid):
    LABEL = "B"
    V_CRUISE = 0.20
    D_SLOW = 0.18
    D_TURN = 0.07
    D_CLEAR = 0.12
    K_W = 1.5


if __name__ == "__main__":
    try:
        AvoidB().run()
    except rospy.ROSInterruptException:
        pass
