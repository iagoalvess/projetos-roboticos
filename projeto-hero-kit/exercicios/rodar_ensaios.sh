#!/usr/bin/env bash
# Roda os ensaios do TP5 sem janela e com semente fixa (uns 35 min). Uso: ./rodar_ensaios.sh
set -u
export MSYS_NO_PATHCONV=1
cd "$(dirname "$0")"
EX=$(pwd -W 2>/dev/null || pwd)
NET=psrensaio SIM=psrensaio-sim ROS=psrensaio-ros

limpa() { docker rm -f "$ROS" "$SIM" >/dev/null 2>&1; }
trap limpa EXIT
docker network inspect "$NET" >/dev/null 2>&1 || docker network create "$NET" >/dev/null

ensaio() {  # cenario script semente duracao_s
  local sc=$1 script=$2 seed=$3 dur=$4 s=0
  limpa
  docker run -d --name "$SIM" --network "$NET" -v psrkit-cache-cpu:/root/.cache psr-kit-sim:cpu \
    python -m psr_sim --scenario "$sc" --robots 1 --backend cpu --seed "$seed" >/dev/null || exit 1
  until docker logs "$SIM" 2>&1 | grep -q "bridge on tcp"; do
    sleep 2; s=$((s + 2))
    [ "$s" -lt 900 ] || { echo "FALHOU: $sc/$script: simulador nao subiu"; exit 1; }
  done
  echo "[$(date +%H:%M:%S)] $sc/$script semente=$seed: simulador em ${s}s"
  docker run -d --name "$ROS" --network "$NET" -v "$EX:/ws/psr_exercicios" \
    -e PSR_SIM_HOST="$SIM" -e PSR_SIM_PORT=11312 psr-kit-ros \
    roslaunch psr_exercicios "$sc.launch" "script:=$script" >/dev/null || exit 1
  s=0
  until docker logs "$ROS" 2>&1 | grep -q "ensaio concluido"; do
    sleep 5; s=$((s + 5))
    [ "$s" -lt $((dur + 120)) ] || { docker logs --tail 20 "$ROS"; echo "FALHOU: $sc/$script: nao concluiu"; exit 1; }
  done
  docker logs "$ROS" 2>&1 | grep "ensaio concluido"
}

ensaio avoid avoid.py 1 300
ensaio avoid avoid_v2.py 1 300
for seed in 1 2 3; do
  ensaio goto goto.py "$seed" 180
  ensaio goto goto_v2.py "$seed" 180
done
echo "RODADA COMPLETA"
