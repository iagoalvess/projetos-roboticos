#!/usr/bin/env bash
# Rodadas de sumo sem janela, em tempo real, com semente fixa.
# Uso: ./rodar_sumo.sh N robot0 robot1 [semente]   (robot = arquivo da pasta sumo/ ou none)
# Ex.: ./rodar_sumo.sh 20 sumo.py oponente_a.py 1
# Grava sumo/log_sumo_<robot0>_x_<robot1>_<data>.csv. Precisa das imagens do ./run.sh build.
set -u
export MSYS_NO_PATHCONV=1
cd "$(dirname "$0")/.."
EX=$(pwd -W 2>/dev/null || pwd)
N=$1 R0=$2 R1=$3 SEED=${4:-1}
NET=psrsumo SIM=psrsumo-sim ROS=psrsumo-ros
limpa() { docker rm -f "$ROS" "$SIM" >/dev/null 2>&1; }
trap limpa EXIT
limpa
docker network inspect "$NET" >/dev/null 2>&1 || docker network create "$NET" >/dev/null
docker run -d --name "$SIM" --network "$NET" -v psrkit-cache-cpu:/root/.cache psr-kit-sim:cpu \
  python -m psr_sim --scenario sumo --robots 2 --backend cpu --seed "$SEED" >/dev/null || exit 1
s=0
until docker logs "$SIM" 2>&1 | grep -q "bridge on tcp"; do
  sleep 2; s=$((s + 2)); [ "$s" -lt 900 ] || { echo "simulador nao subiu"; exit 1; }
done
docker run -d --name "$ROS" --network "$NET" -v "$EX:/ws/psr_exercicios" \
  -e PSR_SIM_HOST="$SIM" -e PSR_SIM_PORT=11312 psr-kit-ros \
  roslaunch psr_exercicios sumo.launch "robot0:=$R0" "robot1:=$R1" >/dev/null || exit 1
sleep 5
docker exec "$ROS" /ros-entrypoint.sh python3 /ws/psr_exercicios/sumo/placar.py "$N" "${R0%.py}_x_${R1%.py}"
