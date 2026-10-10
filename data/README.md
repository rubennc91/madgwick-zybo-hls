# Capturas de la UART

CSV de texto generado por `Integration_Zybo_dual` / `Integration_Zybo_multi` (tecla `L` o `l`).
Cabecera `#HDR` con ejes y offsets, una fila por trama. Se analizan con los scripts de `../analysis/`.

| Fichero | Qué es | Versión del `main.c` | Script |
|---|---|---|---|
| `dual_L_manual.log` | Primera captura del sistema dual (`L`, 60 s), placa a mano, magnetómetro sin calibrar | dual, sin `t_us` | `analyze_dual.py` |
| `dual_l_calibrado.log` | Captura `l` con calibración, placa a mano | dual, sin `t_us` | `analyze_dual.py` |
| `captura_yaw.log`, `captura_tilt.log` | Primeras pruebas con el cubo: giros de 90° en yaw y en vuelcos sobre una arista. Pausas cortas, sin avisos por terminal | dual, sin `t_us` | `analyze_poses.py` |
| `captura_yaw_2.log` | Yaw con pausas demasiado cortas (≈1,4 s); se conserva como ejemplo de captura no válida para el análisis por poses | dual con `t_us` | `analyze_poses.py` |
| `captura_tilt_2.log` | Vuelcos sobre una arista (7 giros, uno de 180°), con `t_us` (tasa real 117,96 Hz) | dual con `t_us` | `analyze_poses.py` |
| `captura_yaw_3.log` | **Yaw con la placa plana: 4 giros de 90° hacia delante y 4 hacia atrás**, pausas de ≈3,4 s | dual con `t_us` y avisos | `analyze_poses.py --gyro` |
| `captura_tilt_3.log` | **Placa en una cara lateral del cubo**, mismos 8 giros sobre la mesa (el eje vertical pasa a ser el Y del sensor) | dual con `t_us` y avisos | `analyze_poses.py --gyro` |

Uso: `python ../analysis/analyze_poses.py captura_yaw_3.log --nominal 90,90,90,90,90,90,90,90 --gyro`

Las capturas de `Integration_Zybo_multi` (varias IMUs) tienen otro formato (columna `imu`, cabecera `#IMU`) y se analizan con `analyze_multi.py`.
Si un `.log` no aparece en `git status`, comprueba qué regla lo ignora: `git check-ignore -v data/captura.log`.
