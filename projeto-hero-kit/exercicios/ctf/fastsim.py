"""Simulador rapido do rouba-bandeira, para testar estrategias em lote.

Usa o juiz do kit (sim/psr_sim/scenarios/ctf.py: posicoes iniciais, segurada, coleta, ponto, empate)
e uma fisica 2D aproximada com as mesmas constantes do world.py: controle de
velocidade com ganho KV_LIN e forca limitada a FORCE_MAX por eixo, contato entre
cilindros com atrito 0,1. Calibrado nos testes de empurrao do sumo (dois a 0,2 m/s
empatam; parado e empurrado a ~0,1 m/s). A observacao chega com LAT s de atraso. Serve para
comparar estrategias; os resultados finais sao do simulador real (rodar_ctf.sh).

Uso (no container do simulador):
  python fastsim.py AZUL VERMELHO [partidas] [duracao_s]   AZUL/VERMELHO = ctf, oponente_a, ..., none
  python fastsim.py torneio ARQ1,ARQ2,... [partidas]       todos contra todos, dos dois lados
  ARQ@PARAM=valor;PARAM=valor troca parametros do arquivo (ex.: ctf@PIN=0)
"""
import importlib
import importlib.util
import math
import sys
from multiprocessing import Pool

from psr_sim.scenarios.ctf import CaptureTheFlag
from psr_sim.world import DT, FORCE_MAX, KV_LIN, KV_YAW, ROBOT_MASS, SUBSTEPS, TORQUE_MAX, V_MAX, W_MAX

import ctf as base

R = 0.037
MU = 0.1
HALF = 0.8
IZZ = 0.5 * ROBOT_MASS * R * R
CTRL = 0.05  # s, os nos rodam a 20 Hz
LAT = float(__import__("os").environ.get("LAT", "0.05"))  # s, atraso da observacao (odom a 25 Hz + ROS)


def clip(x, m):
    return max(-m, min(m, x))


class Body:
    def __init__(self, p):
        self.x, self.y, self.yaw = p.x, p.y, p.yaw
        self.vx = self.vy = self.w = 0.0


def physics(bodies, targets):
    for b, (lin, ang) in zip(bodies, targets):
        fx = clip(KV_LIN * (lin * math.cos(b.yaw) - b.vx), FORCE_MAX)
        fy = clip(KV_LIN * (lin * math.sin(b.yaw) - b.vy), FORCE_MAX)
        tq = clip(KV_YAW * (ang - b.w), TORQUE_MAX)
        b.vx += fx / ROBOT_MASS * DT
        b.vy += fy / ROBOT_MASS * DT
        b.w += tq / IZZ * DT
        b.x += b.vx * DT
        b.y += b.vy * DT
        b.yaw = base.wrap(b.yaw + b.w * DT)
    for _ in range(3):  # contatos: impulso inelastico + atrito, correcao de posicao
        for i in range(len(bodies)):
            a = bodies[i]
            for j in range(i + 1, len(bodies)):
                b = bodies[j]
                dx, dy = b.x - a.x, b.y - a.y
                d = math.hypot(dx, dy)
                if d >= 2 * R or d < 1e-9:
                    continue
                nx, ny = dx / d, dy / d
                vn = (b.vx - a.vx) * nx + (b.vy - a.vy) * ny
                if vn < 0:
                    jn = -vn / 2
                    tx, ty = -ny, nx
                    vt = (b.vx - a.vx) * tx + (b.vy - a.vy) * ty
                    jt = clip(-vt / 2, MU * jn)
                    for s, body in ((-1, a), (1, b)):
                        body.vx += s * (jn * nx + jt * tx)
                        body.vy += s * (jn * ny + jt * ty)
                c = (2 * R - d) / 2
                a.x -= c * nx; a.y -= c * ny
                b.x += c * nx; b.y += c * ny
        for b in bodies:
            lim = HALF - R
            if b.x > lim: b.x, b.vx = lim, min(b.vx, 0.0)
            if b.x < -lim: b.x, b.vx = -lim, max(b.vx, 0.0)
            if b.y > lim: b.y, b.vy = lim, min(b.vy, 0.0)
            if b.y < -lim: b.y, b.vy = -lim, max(b.vy, 0.0)


def load(name):
    """'arquivo' ou 'arquivo@PARAM=valor;PARAM=valor' (copia isolada do modulo com os parametros trocados)."""
    if name == "none":
        return None
    if "@" in name:
        file, params = name.split("@", 1)
        spec = importlib.util.spec_from_file_location("var_%d" % abs(hash(name)), file + ".py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        for kv in params.split(";"):
            k, v = kv.split("=")
            setattr(mod, k, type(getattr(mod, k))(eval(v)))
    else:
        mod = importlib.import_module(name)
    cls = [v for v in vars(mod).values() if isinstance(v, type) and hasattr(v, "step") and v.__module__ == mod.__name__]
    return cls[-1] if cls else base.Brain


def match(blue, red, seed=1, duration=300.0, n=4):
    sc = CaptureTheFlag(n, seed)
    bodies = [Body(p) for p in sc.spawn()]
    roles = ["attacker", "defender"] if n == 4 else ["attacker"]
    teams = [list(range(n // 2)), list(range(n // 2, n))]
    brains = {}
    for t, name in ((0, blue), (1, red)):
        cls = load(name)
        if cls is None:
            continue
        for r, k in zip(roles, teams[t]):
            brains[k] = (cls(k, r, teams[t], teams[1 - t]), 1.0 if t == 0 else -1.0, t)
    cmd = [(0.0, 0.0)] * n
    cmd_t = [-1e9] * n
    pending = None
    t, next_ctrl = 0.0, 0.0
    game = sc.public()
    prev, picks = [None, None], [0, 0]
    hist = []
    while t < duration:
        robots = [{"x": b.x, "y": b.y, "yaw": b.yaw} for b in bodies]
        sc.t = t
        if pending is not None and t >= sc.paused_until:
            for b, p in zip(bodies, pending.poses):
                b.__init__(p)
            sc.round_start = t
            pending = None
            robots = [{"x": b.x, "y": b.y, "yaw": b.yaw} for b in bodies]
        if pending is None:
            ev = sc.referee(robots, None)
            if ev is not None:
                pending = ev
                sc.paused_until = t + ev.pause
        hist.append((t, {k: (b.x, b.y, b.yaw, b.vx * math.cos(b.yaw) + b.vy * math.sin(b.yaw)) for k, b in enumerate(bodies)}, sc.flags(), game))
        while len(hist) > 1 and hist[1][0] <= t - LAT:
            hist.pop(0)
        if t >= next_ctrl:
            next_ctrl += CTRL
            _, pose, flags, gm = hist[0]

            for k, (brain, s, team) in brains.items():
                w = base.team_frame(s, pose, (flags[1 - team]["x"], flags[1 - team]["y"]), gm)
                v, om = brain.step(t, w)
                cmd[k] = (clip(v, V_MAX), clip(s * om, W_MAX))
                cmd_t[k] = t
        frozen = pending is not None
        targets = [(0.0, 0.0) if frozen or sc.held(k) or t - cmd_t[k] > 0.5 else cmd[k] for k in range(n)]
        for _ in range(SUBSTEPS):
            physics(bodies, targets)
            t += DT
        game = sc.public()
        for tm in (0, 1):  # coletas: a bandeira do time tm foi pega
            if sc.carrier[tm] is not None and prev[tm] is None:
                picks[tm] += 1
        prev = list(sc.carrier)
    return {"score": list(sc.score), "tags": list(sc.tags), "picks": picks}


def _job(a):
    return match(*a)


def series(blue, red, games=8, duration=300.0, n=4):
    """Partidas com sementes 1..games, em paralelo."""
    jobs = [(blue, red, s, duration, n) for s in range(1, games + 1)]
    with Pool() as p:
        res = p.map(_job, jobs)
    return res


if __name__ == "__main__":
    if sys.argv[1] == "torneio":
        names = sys.argv[2].split(",")
        games = int(sys.argv[3]) if len(sys.argv) > 3 else 6
        jobs = [(a, b, s, 300.0, 4) for a in names for b in names if a != b for s in range(1, games + 1)]
        with Pool() as p:
            res = p.map(_job, jobs)
        table = {a: {} for a in names}
        for (a, b, *_), r in zip(jobs, res):
            for x, y, sx, sy, px in ((a, b, r["score"][0], r["score"][1], r["picks"][0]), (b, a, r["score"][1], r["score"][0], r["picks"][1])):
                acc = table[x].setdefault(y, [0, 0, 0, 0, 0, 0])  # pontos pro, contra, V, E, D, coletas sofridas
                acc[0] += sx; acc[1] += sy; acc[5] += px
                acc[2 if sx > sy else 3 if sx == sy else 4] += 1
        for a in names:
            line = "  ".join("%s %d-%d (%dV%dE%dD c%d)" % (b, *table[a][b]) for b in names if b != a)
            tot = [sum(table[a][b][i] for b in table[a]) for i in range(6)]
            print("%-8s saldo %+4d  V%d E%d D%d coletas_sofridas %d | %s" % (a, tot[0] - tot[1], tot[2], tot[3], tot[4], tot[5], line))
    else:
        blue, red = sys.argv[1], sys.argv[2]
        games = int(sys.argv[3]) if len(sys.argv) > 3 else 4
        dur = float(sys.argv[4]) if len(sys.argv) > 4 else 300.0
        for s, r in enumerate(series(blue, red, games, dur), 1):
            print("semente %d: azul %d x %d vermelho  segurados %s" % (s, r["score"][0], r["score"][1], r["tags"]))
