#!/bin/bash
# Medida 4: latencia de ida y vuelta con 4 IMU a 952 Hz. En la placa, con las IMU QUIETAS, y con lat_echo.py corriendo en el PC:
#   sudo -i /home/xilinx/bench/bench_latency.sh [carpeta_de_salida]
# Para cada pub_hz de HZS ("10 50 100") arranca el nodo y mide 55 s con lat_board.py. Variables: HZS, ODR (952)
OUT="${1:-/home/xilinx/lat_out_$(date +%Y%m%d_%H%M)}"
HZS="${HZS:-10 50 100}"; ODR="${ODR:-952}"
D=/home/xilinx/bench
mkdir -p "$OUT"
source $D/env.sh
echo "Salida en $OUT"
for hz in $HZS; do
  echo ">> pub_hz $hz (ODR $ODR, 4 IMU)"
  hzf=$(awk -v h="$hz" 'BEGIN{printf "%.1f", h}')    # pub_hz debe llegar como decimal (10.0)
  ( sleep 24; python3 $D/lat_board.py --secs 55 --skip 3 --out "$OUT/lat_hz$hz.csv" --label "ODR $ODR Hz, pub_hz $hz por IMU, 4 IMU" > "$OUT/lat_hz$hz.txt" 2>&1 ) &
  LP=$!
  timeout -s INT 88 /home/xilinx/start_node.sh 4 $ODR $hzf > "$OUT/node_hz$hz.log" 2>&1
  wait $LP
  cat "$OUT/lat_hz$hz.txt" | tee -a "$OUT/lat_summary.txt"
  sleep 5
done
echo "Hecho. Resumen en $OUT/lat_summary.txt"
