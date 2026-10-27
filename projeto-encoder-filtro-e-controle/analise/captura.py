#!/usr/bin/env python3
"""Dispara um ensaio na Mega e grava o registro CSV em ../dados.

python captura.py <comando> [--nome ARQ] [--porta COM3] [--ganhos P1.2 I4 D0]

  comando: n (neutro), a (malha aberta), f (degrau dos filtros),
           d (degrau PID, sintonia), p (patamares), i (inversão)

O arquivo guarda as linhas '#' (condição do ensaio e trocas de passo) e as
amostras t_s,ticks,s_mm,v_raw,v_pb,v_mm,sp,pulso_us até a linha '# fim'.
"""
import argparse
import os
import sys
import time

import serial
from serial.tools import list_ports

sys.stdout.reconfigure(encoding="utf-8")  # console do Windows

NOMES = {
    "n": "neutro",
    "a": "malha_aberta",
    "f": "degrau_filtros",
    "d": "degrau_pid",
    "p": "patamares",
    "i": "inversao",
}
DADOS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "dados")


def encontrar_porta(nome=None):
    if nome:
        return nome
    portas = sorted(list_ports.comports(), key=lambda p: p.device)
    usb = [
        p.device for p in portas
        if p.vid is not None
        or any(chave in p.device.lower() for chave in ("usbmodem", "usbserial", "ttyacm"))
    ]
    if len(usb) == 1:
        return usb[0]
    disponiveis = ", ".join(p.device for p in portas) or "nenhuma"
    raise SystemExit(f"Indique a porta com --porta. Portas disponíveis: {disponiveis}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("comando", choices=sorted(NOMES))
    ap.add_argument("--nome", help="nome do arquivo (sem .csv)")
    ap.add_argument("--porta")
    ap.add_argument("--ganhos", nargs="*", default=[],
                    help="ex.: P1.2 I4 D0 (enviados antes do ensaio)")
    args = ap.parse_args()

    porta = encontrar_porta(args.porta)
    nome = args.nome or NOMES[args.comando]
    os.makedirs(DADOS, exist_ok=True)
    caminho = os.path.join(DADOS, nome + ".csv")

    with serial.Serial(porta, 115200, timeout=1) as ser:
        time.sleep(2)  # a Mega reinicia ao abrir a USB
        ser.reset_input_buffer()
        for g in args.ganhos:
            ser.write((g + "\n").encode())
            time.sleep(1.2)  # parseFloat espera o fim do número
        ser.reset_input_buffer()
        ser.write(args.comando.encode())

        print(f"Gravando {caminho} ...", file=sys.stderr)
        with open(caminho, "w", encoding="utf-8") as arq:
            ocioso = 0
            while True:
                linha = ser.readline().decode(errors="replace").strip()
                if not linha:
                    ocioso += 1
                    if ocioso > 5:
                        raise SystemExit("A placa parou de responder.")
                    continue
                ocioso = 0
                arq.write(linha + "\n")
                print(linha)
                if linha.startswith("# fim"):
                    break


if __name__ == "__main__":
    main()
