#!/usr/bin/env python3
"""Oponente A, a versao anterior: o rouba-bandeira "tradicional" (lei do goto do TP5).

Atacante: vai direto a bandeira inimiga e volta direto para a base.
Defensor: fica entre a propria bandeira e o adversario mais proximo e persegue
quem entra na nossa metade.
"""
import math

from ctf import E, F, Brain, Node, clamp, dist, wrap

K_RHO, K_ALPHA, V_MAX, W_MAX = 1.5, 3.0, 0.2, 4.0


def goto(me, tx, ty):
    rho = math.hypot(tx - me[0], ty - me[1])
    a = wrap(math.atan2(ty - me[1], tx - me[0]) - me[2])
    return clamp(K_RHO * rho * max(0.0, math.cos(a)), 0.0, V_MAX), clamp(K_ALPHA * a, -W_MAX, W_MAX)


class OponenteA(Brain):
    def defend(self, w, me):
        free = [k for k in self.opps if not w["held"][k]]
        if not free:
            return goto(me, F[0] + 0.15, F[1])
        k = min(free, key=lambda k: dist(w["pose"][k], F))
        ox, oy = w["pose"][k][:2]
        if ox < 0.0:
            return goto(me, ox, oy)
        d = max(dist((ox, oy), F), 1e-6)
        return goto(me, F[0] + 0.25 * (ox - F[0]) / d, F[1] + 0.25 * (oy - F[1]) / d)

    def attack(self, w, me):
        if w["carrying"][self.robot]:
            return goto(me, F[0], F[1])
        return goto(me, *w["enemy_flag"])


if __name__ == "__main__":
    Node(OponenteA).run()
