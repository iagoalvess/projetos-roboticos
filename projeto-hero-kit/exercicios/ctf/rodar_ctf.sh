#!/usr/bin/env bash
# Partidas de rouba-bandeira no simulador real (Genesis), sem janela, em tempo real.
# Uso: ./rodar_ctf.sh PARTIDAS AZUL VERMELHO [semente] [id]   (AZUL/VERMELHO = arquivo da pasta ctf/ ou none)
# Ex.: ./rodar_ctf.sh 3 ctf.py oponente_a.py 1
# Cada partida dura 5 min e reinicia o simulador. Grava ctf/log_ctf_<azul>_x_<vermelho>_<data>.csv.
# O id permite rodar varias series em paralelo (containers e rede com nomes distintos).
set -u
export MSYS_NO_PATHCONV=1
cd "$(dirname "$0")/.."
EX=$(pwd -W 2>/dev/null || pwd)
N=$1 B=$2 R=$3 SEED=${4:-1} ID=${5:-0}
NET=psrctf$ID SIM=psrctf$ID-sim ROS=psrctf$ID-ros
limpa() { docker rm -f "$ROS" "$SIM" >/dev/null 2>&1; }
trap 'limpa; docker network rm "$NET" >/dev/null 2>&1' EXIT
docker network inspect "$NET" >/dev/null 2>&1 || docker network create "$NET" >/dev/null
for i in $(seq 1 "$N"); do
  limpa
  docker run -d --name "$SIM" --network "$NET" -v psrkit-cache-cpu:/root/.cache psr-kit-sim:cpu \
    python -m psr_sim --scenario ctf --robots 4 --backend cpu --seed $((SEED + i - 1)) >/dev/null || exit 1
  s=0
  until docker logs "$SIM" 2>&1 | grep -q "bridge on tcp"; do
    sleep 2; s=$((s + 2)); [ "$s" -lt 900 ] || { echo "simulador nao subiu"; exit 1; }
  done
  docker run -d --name "$ROS" --network "$NET" -v "$EX:/ws/psr_exercicios" \
    -e PSR_SIM_HOST="$SIM" -e PSR_SIM_PORT=11312 psr-kit-ros \
    roslaunch psr_exercicios ctf.launch "blue:=$B" "red:=$R" >/dev/null || exit 1
  docker exec "$ROS" /ros-entrypoint.sh python3 /ws/psr_exercicios/ctf/placar.py 300 "${B%.py}_x_${R%.py}" $((SEED + i - 1))
done
