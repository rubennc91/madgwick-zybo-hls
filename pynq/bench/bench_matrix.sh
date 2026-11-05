#!/bin/bash
# Medida 1 y 2: matriz de condiciones con 4 IMU. Ejecutar en la placa con las IMU QUIETAS todo el rato (cada arranque calibra ~8 s):
#   sudo -i /home/xilinx/bench/bench_matrix.sh [carpeta_de_salida]
# Para cada repeticion, condicion (odr/pub_hz) y modo se arranca el nodo DUR segundos:
#   modo A: solo el nodo (da la "CPU libre" de /proc/stat que escribe el propio nodo)
#   modo B: el nodo mas un bucle de fondo de baja prioridad por nucleo (da la CPU libre con la definicion del paper)
# En el PC, bench_sub.py debe estar corriendo todo el tiempo (ver README.md).
# Variables: CONDS ("952/10 952/50 119/10 119/50"), REPS (2), MODES ("A B"), DUR (80), N_IMU (4)
OUT="${1:-/home/xilinx/bench_out_$(date +%Y%m%d_%H%M)}"
CONDS="${CONDS:-952/10 952/50 119/10 119/50}"
REPS="${REPS:-2}"; MODES="${MODES:-A B}"; DUR="${DUR:-80}"; N_IMU="${N_IMU:-4}"
D=/home/xilinx/bench
mkdir -p "$OUT"
source /etc/profile.d/pynq_venv.sh
echo "Salida en $OUT"
echo "Plan: REPS=$REPS CONDS=$CONDS MODES=$MODES DUR=${DUR}s N_IMU=$N_IMU" | tee "$OUT/plan.txt"
echo "run,rep,cond,mode,odr,hz,t_start,node_log,bg_csv" > "$OUT/runs.csv"
echo ">> referencia inicial sin nodo (35 s)"
python3 $D/bg_loop.py --secs 35 --out "$OUT/base_start.csv"
n=0
for r in $(seq 1 $REPS); do
  for c in $CONDS; do
    for m in $MODES; do
      n=$((n+1)); odr=${c%/*}; hz=${c#*/}
      hzf=$(awk -v h="$hz" 'BEGIN{printf "%.1f", h}')    # pub_hz debe llegar como decimal (10.0), si no rclpy lo rechaza
      id=$(printf "%02d_%s_%s_r%d" $n "${c/\//-}" $m $r)
      echo ">> $id: ODR $odr Hz, pub_hz $hz, modo $m, $N_IMU IMU"
      BGPID=""; BGF=""
      if [ "$m" = "B" ]; then
        BGF="bg_$id.csv"
        python3 $D/bg_loop.py --secs $((DUR+5)) --out "$OUT/$BGF" > /dev/null &
        BGPID=$!
      fi
      t0=$(date +%s.%N)
      timeout -s INT $DUR /home/xilinx/start_node.sh $N_IMU $odr $hzf > "$OUT/node_$id.log" 2>&1
      [ -n "$BGPID" ] && wait $BGPID
      echo "$n,$r,$c,$m,$odr,$hz,$t0,node_$id.log,$BGF" >> "$OUT/runs.csv"
      sleep 6
    done
  done
done
echo ">> referencia final sin nodo (35 s)"
python3 $D/bg_loop.py --secs 35 --out "$OUT/base_end.csv"
python3 $D/bench_summary.py "$OUT" | tee "$OUT/summary.txt"
echo "Hecho. Resumen en $OUT/summary.txt"
