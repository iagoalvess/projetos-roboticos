#!/usr/bin/env python3
"""Segundo conjunto de ganhos: giro lento."""
import rospy

from goto import Goto


class GotoB(Goto):
    LABEL = "B"
    K_ALPHA = 1.0


if __name__ == "__main__":
    try:
        GotoB().run()
    except rospy.ROSInterruptException:
        pass
