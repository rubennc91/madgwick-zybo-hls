# `ros2 topic hz /madgwick/q` (resumen)

Resumen hecho a mano de las salidas pegadas durante la medida (ventanas de 15 a 30 s). Al publicar las 4 IMU juntas en cada periodo, el intervalo mínimo entre mensajes es 0 y el máximo se acerca al periodo de publicación.

| ODR | Mensajes/s nominales | Tasa media observada (Hz) | Intervalo máximo entre mensajes |
|---|---|---|---|
| 119 Hz | 40 | 39,99 a 40,25 | 0,105 s |
| 119 Hz | 200 | 199,999 a 200,36 | 0,028 s |
| 952 Hz | 40 | 39,98 a 40,19 | 0,110 s |
| 952 Hz | 200 | 199,64 a 199,998 | 0,046 s |

En la prueba de 952 Hz y 200 mensajes/s, los publicados (5 992) más las lecturas descartadas (8) suman 6 000 = 30 s × 200 mensajes/s; la tasa media de 199,7 Hz coincide con 200 × 5 992 / 6 000.

Con la lectura repetida (952 Hz, 200 mensajes/s, 9 600 publicados): tasa media entre 199,9 y 200,2 Hz tras la primera ventana (201,47 Hz en los primeros 2 s), intervalo máximo 0,024 s. Copiado a mano; la salida pegada correspondía a la ejecución del reintento.
