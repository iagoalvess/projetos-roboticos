#!/usr/bin/env python3
"""Refaz os gráficos e as métricas do relatório a partir dos registros.

python graficos.py

Lê ../dados/*.csv (gravados por captura.py) e escreve as figuras em
../relatorio/relatorio-4/imagens e um resumo numérico em ../dados/resumo.txt.
Ensaios sem arquivo são pulados. Os registros antigos ../velocidade*.txt
(firmware anterior, só um sentido) geram a curva preliminar.
"""
import glob
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.stdout.reconfigure(encoding="utf-8")  # console do Windows

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.join(AQUI, "..")
DADOS = os.path.join(RAIZ, "dados")
IMAGENS = os.path.join(RAIZ, "relatorio", "relatorio-4", "imagens")

U_MAX = 250       # us, mesmo limite do firmware
DESCARTE_S = 1.0  # transitório descartado em cada passo da malha aberta
REGIME_S = 1.0    # janela final de cada patamar usada como regime

COR_SP = "#222222"
COR_BRUTA = "#9a9a9a"
COR_PB = "#1f5fa8"
COR_MM = "#c2410c"
COR_PULSO = "#4d7c0f"

resumo = []


def nota(texto=""):
    print(texto)
    resumo.append(texto)


def ler(nome):
    """Devolve (meta, dados) de ../dados/<nome>.csv, ou None se não existir."""
    caminho = os.path.join(DADOS, nome + ".csv")
    if not os.path.exists(caminho):
        print(f"[pulado] {caminho} não existe")
        return None
    meta, linhas = {}, []
    with open(caminho, encoding="utf-8") as arq:
        for linha in arq:
            linha = linha.strip()
            if linha.startswith("# ensaio="):
                meta = dict(re.findall(r"(\w+)=(\S+)", linha))
            elif linha and not linha.startswith(("#", "t_s")):
                partes = linha.split(",")
                if len(partes) == 8:
                    linhas.append([float(x) for x in partes])
    d = np.array(linhas)
    cols = ["t", "ticks", "s", "v", "v_pb", "v_mm", "sp", "pulso"]
    dados = {c: d[:, j] for j, c in enumerate(cols)}
    filtro = meta.get("filtro_pid", "passa-baixa")
    dados["vf"] = dados["v_pb"] if filtro == "passa-baixa" else dados["v_mm"]
    dt = np.diff(dados["t"]) * 1000
    k = float(meta.get("k_mm_tick", "nan"))
    nota(f"== {nome}: Δt real = {dt.mean():.2f} ± {dt.std():.2f} ms "
         f"(mín {dt.min():.0f}, máx {dt.max():.0f}); degrau k/Δt = "
         f"{k / (dt.mean() / 1000):.2f} mm/s")
    return meta, dados


def condicao(meta):
    return (f"neutro = {meta.get('neutro_us')} µs, k = {meta.get('k_mm_tick')} mm/tick, "
            f"Δt = {meta.get('dt_ms')} ms, kp = {meta.get('kp')}, ki = {meta.get('ki')}, "
            f"kd = {meta.get('kd')}, filtro = {meta.get('filtro_pid')} "
            f"(α = {meta.get('alfa')}, janela = {meta.get('janela')})")


def salvar(fig, nome):
    os.makedirs(IMAGENS, exist_ok=True)
    fig.tight_layout()
    fig.savefig(os.path.join(IMAGENS, nome + ".pdf"))
    fig.savefig(os.path.join(IMAGENS, nome + ".png"), dpi=150)
    plt.close(fig)


def blocos(valores):
    """Índices [ini, fim) dos trechos em que 'valores' fica constante."""
    troca = np.flatnonzero(np.diff(valores) != 0) + 1
    limites = np.concatenate(([0], troca, [len(valores)]))
    return list(zip(limites[:-1], limites[1:]))


def media_por_passo(meta, d):
    """Velocidade média em regime para cada pulso aplicado."""
    pulsos, velocidades = [], []
    for ini, fim in blocos(d["pulso"]):
        t0 = d["t"][ini]
        sel = slice(ini, fim)
        regime = d["t"][sel] >= t0 + DESCARTE_S
        if regime.sum() < 3:
            continue
        pulsos.append(d["pulso"][ini])
        velocidades.append(d["v"][sel][regime].mean())
    return np.array(pulsos), np.array(velocidades)


# ---------------------------------------------------------------------------
def neutro():
    r = ler("neutro")
    if r is None:
        return
    meta, d = r
    p, v = media_por_passo(meta, d)
    nota("== Busca do neutro")
    for pi, vi in zip(p, v):
        nota(f"  {pi:6.0f} us -> {vi:8.2f} mm/s")
    fig, ax = plt.subplots(figsize=(6, 3.2))
    ax.plot(p, v, "o-", color=COR_PB)
    ax.axhline(0, color="k", lw=0.6)
    ax.axvline(float(meta["neutro_us"]), color="#b91c1c", ls="--",
               label=f"neutro adotado = {meta['neutro_us']} µs")
    ax.set_xlabel("pulso (µs)")
    ax.set_ylabel("velocidade média (mm/s)")
    ax.legend()
    ax.grid(alpha=0.3)
    salvar(fig, "neutro")


def malha_aberta():
    r = ler("malha_aberta")
    if r is None:
        return
    meta, d = r
    p, v = media_por_passo(meta, d)
    neutro_us = float(meta["neutro_us"])
    nota("== Curva em malha aberta (regime, após descartar "
         f"{DESCARTE_S:.1f} s de cada passo)")
    for pi, vi in zip(p, v):
        nota(f"  {pi:6.0f} us ({pi - neutro_us:+5.0f}) -> {vi:8.2f} mm/s")
    fig, ax = plt.subplots(figsize=(6.5, 3.8))
    ax.plot(p, v, "o-", color=COR_PB, label="velocidade em regime")
    ax.axvline(neutro_us, color="#b91c1c", ls="--",
               label=f"neutro = {neutro_us:.0f} µs")
    ax.axvspan(neutro_us - U_MAX, neutro_us + U_MAX, color="#b91c1c", alpha=0.06,
               label="faixa do PID (neutro ± 250 µs)")
    ax.axhline(0, color="k", lw=0.6)
    ax.set_xlabel("pulso (µs)")
    ax.set_ylabel("velocidade (mm/s)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    salvar(fig, "curva_aberta")


def curva_preliminar():
    """Registros antigos: ticks,dist_cm,dt_s,v_cm_s,vf_cm_s (só quando havia giro)."""
    arquivos = sorted(glob.glob(os.path.join(RAIZ, "velocidade*.txt")))
    if not arquivos:
        return
    k_ref = np.pi * 49.2 / 288  # mm/tick do mecanismo de referência
    p, tps = [], []
    for caminho in arquivos:
        pulso = int(re.search(r"(\d+)", os.path.basename(caminho)).group(1))
        linhas = []
        with open(caminho) as arq:
            for linha in arq:
                partes = linha.strip().rstrip(";").split(",")
                if len(partes) == 5:
                    linhas.append([float(x) for x in partes])
        a = np.array(linhas)[1:]  # a primeira linha vem do prevpos = -999
        a = a[len(a) // 3:]       # descarta o arranque
        tps.append((a[-1, 0] - a[0, 0]) / a[1:, 2].sum())
        p.append(pulso)
    p, tps = np.array(p), np.array(tps)
    nota("== Curva preliminar (firmware antigo, k de referência)")
    for pi, ti in zip(p, tps):
        nota(f"  {pi:6.0f} us -> {ti:7.1f} ticks/s = {ti * k_ref:7.1f} mm/s")
    fig, ax = plt.subplots(figsize=(6, 3.2))
    ax.plot(p, tps * k_ref, "o-", color=COR_PB)
    ax.set_xlabel("pulso (µs)")
    ax.set_ylabel("velocidade (mm/s)")
    ax.set_title("Ensaio preliminar (k = 0,537 mm/tick de referência)", fontsize=9)
    ax.grid(alpha=0.3)
    salvar(fig, "curva_preliminar")


def filtros():
    r = ler("degrau_filtros")
    if r is None:
        return
    meta, d = r
    t = d["t"]
    fig, ax = plt.subplots(figsize=(6.5, 3.8))
    ax.step(t, d["v"], where="post", color=COR_BRUTA, lw=1, label="bruta (Δs/Δt)")
    ax.plot(t, d["v_pb"], color=COR_PB, lw=1.6, label=f"passa-baixa (α = {meta['alfa']})")
    ax.plot(t, d["v_mm"], color=COR_MM, lw=1.6,
            label=f"média móvel (janela = {meta['janela']})")
    ax.set_xlabel("tempo (s)")
    ax.set_ylabel("velocidade (mm/s)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    salvar(fig, "filtros")

    # Ruído: desvio padrão no fim do degrau. Atraso: tempo até 63 % do regime.
    (ini, fim) = max(blocos(d["pulso"]), key=lambda b: d["pulso"][b[0]])
    t0 = t[ini]
    regime = (t >= t[fim - 1] - 2.0) & (t <= t[fim - 1])
    nota("== Filtros no degrau (pulso "
         f"{d['pulso'][ini]:.0f} us a partir de t = {t0:.2f} s)")
    for nome, sinal in (("bruta", d["v"]), ("passa-baixa", d["v_pb"]),
                        ("media-movel", d["v_mm"])):
        final = sinal[regime].mean()
        desvio = sinal[regime].std()
        apos = np.flatnonzero((t >= t0) & (sinal >= 0.63 * final))
        t63 = t[apos[0]] - t0 if len(apos) else float("nan")
        nota(f"  {nome:12s} regime = {final:7.2f} mm/s  desvio = {desvio:6.2f} mm/s"
             f"  t63 = {t63:5.2f} s")


def grafico_malha(meta, d, nome, titulo):
    t = d["t"]
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(6.5, 5.2), sharex=True,
                                  gridspec_kw={"height_ratios": [3, 1.3]})
    ax.step(t, d["v"], where="post", color=COR_BRUTA, lw=0.8, label="velocidade bruta")
    ax.plot(t, d["vf"], color=COR_PB, lw=1.6, label="velocidade filtrada")
    ax.step(t, d["sp"], where="post", color=COR_SP, ls="--", lw=1.4, label="setpoint")
    ax.set_ylabel("velocidade (mm/s)")
    ax.set_title(titulo, fontsize=10)
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    neutro_us = float(meta["neutro_us"])
    ax2.plot(t, d["pulso"], color=COR_PULSO, lw=1.2)
    ax2.axhline(neutro_us, color="k", lw=0.6)
    for lim in (neutro_us - U_MAX, neutro_us + U_MAX):
        ax2.axhline(lim, color="#b91c1c", ls=":", lw=1)
    ax2.set_ylabel("pulso (µs)")
    ax2.set_xlabel("tempo (s)")
    ax2.grid(alpha=0.3)
    salvar(fig, nome)


def metricas_patamar(meta, d, ini, fim):
    t, sp, vf = d["t"], d["sp"][ini], d["vf"]
    seg = slice(ini, fim)
    ts, vs = t[seg], vf[seg]
    t0 = ts[0]
    regime = ts >= ts[-1] - REGIME_S
    v_reg = vs[regime].mean()
    erro = sp - v_reg
    # tempo de subida 10-90 % relativo à variação de setpoint
    sp_ant = d["sp"][ini - 1] if ini > 0 else 0.0
    dv = sp - sp_ant
    def cruza(frac):
        alvo = sp_ant + frac * dv
        idx = np.flatnonzero((vs - alvo) * np.sign(dv) >= 0)
        return ts[idx[0]] - t0 if len(idx) else float("nan")
    t10, t90 = cruza(0.1), cruza(0.9)
    pico = vs.max() if dv > 0 else vs.min()
    sobre = max(0.0, (pico - sp) * np.sign(dv)) / abs(dv) * 100 if dv else 0.0
    satur = np.mean(np.abs(d["pulso"][seg] - float(meta["neutro_us"])) >= U_MAX) * 100
    return dict(sp=sp, v_reg=v_reg, erro=erro, t10=t10, t90=t90, sobre=sobre,
                satur=satur, desvio=vs[regime].std())


def relatorio_malha(meta, d, rotulo):
    nota(f"  condição: {condicao(meta)}")
    for ini, fim in blocos(d["sp"]):
        if d["sp"][ini] == 0:
            continue
        m = metricas_patamar(meta, d, ini, fim)
        nota(f"  sp = {m['sp']:7.1f}  regime = {m['v_reg']:7.2f} ± {m['desvio']:5.2f}"
             f"  erro = {m['erro']:6.2f} mm/s  subida(10-90%) = {m['t90'] - m['t10']:5.2f} s"
             f"  t90 = {m['t90']:5.2f} s  sobressinal = {m['sobre']:5.1f} %"
             f"  saturado = {m['satur']:5.1f} %")
    nota(f"  distância final = {d['s'][-1]:.1f} mm "
         f"({d['ticks'][-1]:.0f} ticks, {d['ticks'][-1] / float(meta['N']):.2f} voltas)")


def sintonia():
    arquivos = sorted(glob.glob(os.path.join(DADOS, "degrau_pid*.csv")))
    if not arquivos:
        print("[pulado] nenhum degrau_pid*.csv")
        return
    nota("== Sintonia (degraus em malha fechada)")
    fig, ax = plt.subplots(figsize=(6.5, 3.8))
    for caminho in arquivos:
        nome = os.path.splitext(os.path.basename(caminho))[0]
        meta, d = ler(nome)
        nota(f"-- {nome}")
        relatorio_malha(meta, d, nome)
        ax.plot(d["t"], d["vf"], lw=1.3,
                label=f"kp={meta['kp']} ki={meta['ki']} kd={meta['kd']}")
        sp = d["sp"]
    ax.step(d["t"], sp, where="post", color=COR_SP, ls="--", label="setpoint")
    ax.set_xlabel("tempo (s)")
    ax.set_ylabel("velocidade filtrada (mm/s)")
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)
    salvar(fig, "sintonia")


def patamares():
    r = ler("patamares")
    if r is None:
        return None
    meta, d = r
    nota("== Cinco patamares")
    relatorio_malha(meta, d, "patamares")
    grafico_malha(meta, d, "patamares", "Cinco patamares")
    return d


def patamares_v1(d_pat):
    # primeiro ensaio, com a integral zerada a cada troca de patamar
    r = ler("patamares_v1")
    if r is None or d_pat is None:
        return
    meta, d = r
    nota("== Cinco patamares, versão 1 (integral zerada a cada patamar)")
    relatorio_malha(meta, d, "patamares_v1")
    fig, ax = plt.subplots(figsize=(6.5, 3.6))
    ax.plot(d["t"], d["vf"], color=COR_PB, lw=1.3,
            label=f"integral zerada a cada patamar (final = {d['s'][-1]:.0f} mm)")
    ax.plot(d_pat["t"], d_pat["vf"], color=COR_MM, lw=1.3,
            label=f"integral mantida (final = {d_pat['s'][-1]:.0f} mm)")
    ax.step(d_pat["t"], d_pat["sp"], where="post", color=COR_SP, ls="--", label="setpoint")
    ax.set_xlabel("tempo (s)")
    ax.set_ylabel("velocidade filtrada (mm/s)")
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)
    salvar(fig, "patamares_v1")


def inversao():
    r = ler("inversao")
    if r is None:
        return None
    meta, d = r
    nota("== Inversão de sentido")
    relatorio_malha(meta, d, "inversao")
    # tempo em que a roda fica parada na passagem pelo zero
    neg = np.flatnonzero(d["sp"] < 0)
    if len(neg):
        t_troca = d["t"][neg[0]]
        janela = neg[(d["t"][neg] - t_troca) < 2.0]
        parado = np.sum(d["v"][janela] == 0) * float(meta["dt_ms"]) / 1000
        nota(f"  tempo com v_bruta = 0 após a troca: {parado:.2f} s")
    grafico_malha(meta, d, "inversao", "Inversão de sentido")
    return d


def distancia(d_pat, d_inv):
    if d_pat is None and d_inv is None:
        return
    fig, ax = plt.subplots(figsize=(6.5, 3.4))
    for d, rotulo, cor in ((d_pat, "cinco patamares", COR_PB),
                           (d_inv, "inversão", COR_MM)):
        if d is not None:
            ax.plot(d["t"], d["s"], color=cor, lw=1.6,
                    label=f"{rotulo} (final = {d['s'][-1]:.0f} mm)")
    ax.set_xlabel("tempo (s)")
    ax.set_ylabel("distância s (mm)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    salvar(fig, "distancia")


def main():
    neutro()
    malha_aberta()
    curva_preliminar()
    filtros()
    sintonia()
    d_pat = patamares()
    patamares_v1(d_pat)
    d_inv = inversao()
    distancia(d_pat, d_inv)
    os.makedirs(DADOS, exist_ok=True)
    with open(os.path.join(DADOS, "resumo.txt"), "w", encoding="utf-8") as arq:
        arq.write("\n".join(resumo) + "\n")


if __name__ == "__main__":
    main()
