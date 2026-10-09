#!/usr/bin/env python3
"""Plota ao vivo a serial da Mega.
Uma coluna (o sinal cru): desenha contagens.
Duas colunas sinal,mm: desenha a distância em milímetros.
python3 plot_serial.py [/dev/cu.usbmodem...]
"""
import os
import sys
import time
from collections import deque
import matplotlib.pyplot as plt
import serial
from serial.tools import list_ports


def encontrar_porta(nome=None):
    portas = sorted(porta.device for porta in list_ports.comports())
    usb = [
        porta for porta in portas
        if any(chave in porta.lower() for chave in ("usbmodem", "usbserial", "ttyacm"))
    ]

    if nome and os.path.exists(nome):
        return nome

    if nome:
        correspondencias = [porta for porta in portas if porta.startswith(nome)]
        if len(correspondencias) == 1:
            return correspondencias[0]

    if len(usb) == 1:
        if nome:
            print(f"Porta {nome!r} não encontrada; usando {usb[0]}.", file=sys.stderr)
        return usb[0]

    disponiveis = ", ".join(portas) or "nenhuma"
    raise SystemExit(
        "Não foi possível identificar a porta da placa. "
        f"Portas disponíveis: {disponiveis}"
    )


porta = encontrar_porta(sys.argv[1] if len(sys.argv) > 1 else None)
try:
    ser = serial.Serial(porta, 115200, timeout=1)
except serial.SerialException as erro:
    raise SystemExit(f"Não foi possível abrir {porta}: {erro}") from erro

print(f"Lendo {porta} a 115200 baud.", file=sys.stderr)
time.sleep(2) # a Mega reinicia ao abrir a USB
n = 0
xs, ys = deque(maxlen=200), deque(maxlen=200)
modo = None
plt.ion()
fig, ax = plt.subplots()
(linha,) = ax.plot([], [], color="#960B22", lw=1.5)
ax.set_xlabel("amostra")
fig.tight_layout()

def amostra(bruto):
    if bruto.isdigit():
        return "cru", float(bruto)
    partes = bruto.split(",")
    if len(partes) == 2:
        try:
            return "mm", float(partes[1])
        except ValueError:
            return None
    return None

try:
    while True:
        bruto = ser.readline().decode(errors="ignore").strip()
        lido = amostra(bruto)
        if lido is None:
            continue
        kind, valor = lido
        if modo is None:
            modo = kind
        if modo == "mm":
            ax.set_ylabel("distância (mm)")
        else:
            ax.set_ylabel("sinal (contagens)")
        if kind != modo:
            continue
        n += 1
        xs.append(n)
        ys.append(valor)
        print(bruto, flush=True)
        linha.set_data(xs, ys)
        ax.relim()
        ax.autoscale_view()
        plt.pause(0.001)
except KeyboardInterrupt:
    pass
finally:
    ser.close()
