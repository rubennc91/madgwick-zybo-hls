#!/usr/bin/env python3
"""Diagnostico de las 4 IMU: arranca cada nav_spi_ctrl por separado y muestra STATUS y WHO_AM_I.
Uso (en la placa):  sudo -i python3 /home/xilinx/nav_diag.py [ruta_al_.bit]"""
import sys, time
from pynq import Overlay, MMIO
BIT = sys.argv[1] if len(sys.argv) > 1 else '/home/xilinx/jupyter_notebooks/zybo/zybo_multi.bit'
PUERTO = ['JE', 'JD', 'JC', 'JA']
Overlay(BIT)
for i in range(4):
    m = MMIO(0x40000000 + 0x10000 * i, 0x1000)
    m.write(0x04, (7 << 6) | (6 << 3) | 6)   # ODR 952 Hz como en el nodo
    m.write(0x00, 1)
    t0 = time.time()
    while not (m.read(0x40) & 0x12) and time.time() - t0 < 2:
        time.sleep(0.001)
    st, who = m.read(0x40), m.read(0x48)
    print('IMU%d (%s): STATUS=0x%08x  cfg_done=%d ag=%d mag=%d err=%d [whoami_ag=%d whoami_mag=%d timeout=%d overrun=%d]  WHO A/G=0x%02x (0x68) Mag=0x%02x (0x3D)' % (
        i, PUERTO[i], st, st >> 1 & 1, st >> 2 & 1, st >> 3 & 1, st >> 4 & 1, st >> 5 & 1, st >> 6 & 1, st >> 7 & 1, st >> 8 & 1, who & 0xFF, who >> 8 & 0xFF))
