#!/usr/bin/env python3
"""Oponente B: a nossa defesa (tampa sobre a bandeira),
com um atacante a toda que vai e volta em linha reta, sem desviar."""
from ctf import F, Brain, Node, drive


class OponenteB(Brain):
    def attack(self, w, me):
        if w["carrying"][self.robot]:
            return drive(me, F[0] + 0.09, F[1])
        return drive(me, *w["enemy_flag"])


if __name__ == "__main__":
    Node(OponenteB).run()
