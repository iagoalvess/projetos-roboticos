#!/usr/bin/env python3
"""Metricas e graficos dos ensaios. Uso: python3 analise.py avoid/log_*.csv goto/log_*.csv"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

COLLISION = 0.045  # m, definido no enunciado
CELL = 0.10  # m


def wall(ax):
    ax.plot([-1, 1, 1, -1, -1], [-1, -1, 1, 1, -1], "k", lw=2)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


def legend_below(ax):
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=2, frameon=False)


def area(d):
    return np.unique(np.floor(np.c_[d["x"], d["y"]] / CELL), axis=0).shape[0] * CELL ** 2


def avoid(d, name):
    dist = np.hypot(np.diff(d["x"]), np.diff(d["y"])).sum()
    below = d["min_range"] < COLLISION
    collisions = int(below[0]) + int(np.sum(below[1:] & ~below[:-1]))  # entradas, nao amostras
    late = d["t"] >= d["t"][-1] - 100
    print("%s: duracao=%.1f s  distancia=%.2f m  colisoes=%d  menor leitura=%.3f m  "
          "area coberta=%.2f m2 (ultimos 100 s: %.2f m2)" % (
              name, d["t"][-1], dist, collisions, d["min_range"].min(), area(d), area(d[late])))

    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4.5))
    wall(a)
    a.add_patch(plt.Circle((0, 0), 0.15, color="tab:orange"))
    a.plot(d["x"], d["y"], lw=0.7)
    a.plot(d["x"][0], d["y"][0], "go", label="inicio")
    legend_below(a)
    a.set_title("Trajetoria")
    b.plot(d["t"], d["min_range"], lw=0.7)
    b.axhline(COLLISION, color="r", ls="--", label="colisao (0,045 m)")
    b.set_xlabel("t (s)")
    b.set_ylabel("menor distancia (m)")
    b.legend()
    b.set_title("Menor distancia lida")
    return fig


def goto(d, name):
    rho = np.hypot(d["gx"] - d["x"], d["gy"] - d["y"])
    alpha = np.arctan2(d["gy"] - d["y"], d["gx"] - d["x"]) - d["yaw"]
    alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
    hit = np.flatnonzero(np.diff(d["reached"]) > 0) + 1
    mean = np.diff(np.r_[0.0, d["t"][hit]]).mean() if hit.size else float("nan")
    print("%s: duracao=%.1f s  metas=%d  tempo medio=%.2f s" % (name, d["t"][-1], d["reached"][-1], mean))

    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4.5))
    wall(a)
    a.plot(d["x"], d["y"], lw=0.7)
    a.plot(d["x"][0], d["y"][0], "go", label="inicio")
    a.plot(d["gx"][hit - 1], d["gy"][hit - 1], "rx", label="metas atingidas")
    legend_below(a)
    a.set_title("Trajetoria")
    b.plot(d["t"], rho, lw=0.7, label="rho (m)")
    b.plot(d["t"], alpha, lw=0.7, label="alpha (rad)")
    b.set_xlabel("t (s)")
    b.legend()
    b.set_title("Erros ao longo do tempo")
    return fig


for path in sys.argv[1:]:
    data = np.genfromtxt(path, delimiter=",", names=True)
    if data.size < 2:
        sys.exit("%s: ensaio vazio" % path)
    name = os.path.basename(path)
    fig = goto(data, name) if "gx" in data.dtype.names else avoid(data, name)
    fig.suptitle(name)
    fig.tight_layout()
    fig.savefig(os.path.splitext(path)[0] + ".png", dpi=150)
    plt.close(fig)
