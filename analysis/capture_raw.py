#!/usr/bin/env python3
"""Captura el streaming CRUDO de main.c (tecla 'w') a un CSV, para repetir el filtro fuera de linea.
Uso:   python capture_raw.py COM5 921600 deriva_raw.csv [--minutes 60] [--wait 10] [--send w]
       python capture_raw.py --selftest
Antes: pon ODR_CODE 3 y N_IMU 2 en main.c (el modo 'w' exige ODR_SCALE == 1).
Fases: ESPERA (--wait s, para que se estabilice el sistema) -> envia 'w' -> CAPTURA (cada 30 s una linea de progreso)
       -> con Ctrl-C o al cumplirse --minutes envia una tecla para parar y cierra el CSV.
Registro: A5 5C | imu | flags | cnt(u32) | t_us(u32) | 9 x int32 (g a m) | 4 x float32 (q) | temp(int16) | crc8(poly 0x07)   (67 B)
          A5 5B = igual sin temperatura (65 B, firmware antiguo); se acepta y la columna temp_raw queda vacia (nan)
CSV: primera linea '# RAW,...' (cabecera del firmware) y luego
     host_s,imu,flags,cnt,t_s,w0..w8,q0..q3,temp_c     (t_s = t_us desenrollado: el contador de 32 bits da la vuelta a los 71,6 min)"""
import sys, re, struct, time, argparse
import numpy as np

def crc8(data):
    c = 0
    for b in data:
        c ^= b
        for _ in range(8): c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c

REC_B = 2 + 1 + 1 + 4 + 4 + 36 + 16 + 1      # 65 bytes (A5 5B)
REC_C = REC_B + 2                            # 67 bytes (A5 5C: + temperatura int16)
class RawParser:
    def __init__(s): s.buf = bytearray(); s.good = 0; s.bad = 0; s.header = None; s.t_last = {}; s.wraps = {}; s.cnt_last = {}; s.gaps = {}; s.lost = {}
    def feed(s, d):
        s.buf += d; out = []
        if s.header is None:
            m = re.search(rb'#RAW,[^\r\n]*\r?\n', bytes(s.buf[:4096]))      # solo cuando la linea esta COMPLETA
            if m: s.header = m.group(0).decode(errors='replace').strip()
        while True:
            iB = s.buf.find(b'\xA5\x5B'); iC = s.buf.find(b'\xA5\x5C')
            cand = [x for x in (iB, iC) if x >= 0]
            i = min(cand) if cand else -1
            if i < 0:
                if s.header is None and len(s.buf) < 8192: break      # la cabecera de texto puede llegar troceada: no tirar el buffer
                del s.buf[:-1]; break
            if i: del s.buf[:i]
            REC = REC_C if s.buf[1] == 0x5C else REC_B
            if len(s.buf) < REC: break
            body = bytes(s.buf[2:REC-1])
            if crc8(body) != s.buf[REC-1] or body[0] > 3:
                s.bad += 1; del s.buf[:2]; continue
            imu, flags, cnt, t_us = struct.unpack('<BBII', body[:10])
            w = struct.unpack('<9i', body[10:46]); q = struct.unpack('<4f', body[46:62])
            tl = s.t_last.get(imu)
            if tl is not None and t_us < tl - 2**31: s.wraps[imu] = s.wraps.get(imu, 0) + 1
            s.t_last[imu] = t_us
            t_s = (t_us + s.wraps.get(imu, 0) * 2**32) / 1e6
            cl = s.cnt_last.get(imu)
            if cl is not None and cnt != cl + 1:
                s.gaps[imu] = s.gaps.get(imu, 0) + 1; s.lost[imu] = s.lost.get(imu, 0) + (cnt - cl - 1)
            temp = struct.unpack('<h', body[62:64])[0] if REC == REC_C else None
            s.cnt_last[imu] = cnt; s.good += 1; del s.buf[:REC]
            out.append((imu, flags, cnt, t_s, w, q, temp))
        return out

def make_rec(imu, flags, cnt, t_us, w, q, temp=None):
    body = struct.pack('<BBII', imu, flags, cnt, t_us) + struct.pack('<9i', *w) + struct.pack('<4f', *q)
    if temp is None: return b'\xA5\x5B' + body + bytes([crc8(body)])
    body += struct.pack('<h', temp)
    return b'\xA5\x5C' + body + bytes([crc8(body)])

def selftest():
    rng = np.random.default_rng(0); data = b'\r\n#RAW,fmt=A55B,n=2,odr_hz=119,fs_g=1,dt_ns=8280000/8307000/0/0,beta_start=1.0,beta_run=0.1,beta_start_ms=3000,gt_hz=333335658\r\n'
    recs = []; cnt = [100, 200]; t = [0, 0]
    for k in range(400):
        for imu in (0, 1):
            cnt[imu] += 1 + (1 if k == 150 and imu == 1 else 0)                    # una trama no leida en la IMU 1
            t[imu] = (t[imu] + 8300) % 2**32
            w = [int(v) for v in rng.integers(-20000, 20000, 9)]; q = [float(v) for v in rng.normal(0, 1, 4)]
            recs.append(make_rec(imu, 1, cnt[imu], t[imu], w, q, temp=(-100 + k if k >= 200 else None)))
    data += b''.join(recs); data = bytearray(data); data[700] ^= 0xFF; data = bytes(data)     # un byte corrupto
    p = RawParser(); out = []
    for i in range(0, len(data), 53): out += p.feed(data[i:i+53])
    print('registros buenos=%d malos=%d huecos=%s perdidas=%s cabecera=%s' % (p.good, p.bad, p.gaps, p.lost, 'ok' if p.header and p.header.startswith('#RAW,fmt=A55B') else p.header))
    assert p.good == 799 and p.bad >= 1 and p.lost.get(1) == 1 and p.header
    temps = [r[6] for r in out if r[6] is not None]; print('con temperatura: %d registros, primero %s ultimo %s (raw)' % (len(temps), temps[0], temps[-1])); assert len(temps) > 300 and temps[0] >= 100 - 100
    # desenrollado de t_us (32 bits)
    p2 = RawParser(); o = []
    for k, tu in enumerate([4294960000, 4294967000, 1000, 9000]): o += p2.feed(make_rec(0, 1, k, tu, [0]*9, [1, 0, 0, 0], temp=-5 if k == 3 else None))
    ts = [r[3] for r in o]; print('t desenrollado:', ts); assert all(np.diff(ts) > 0)
    print('OK')

def main():
    if '--selftest' in sys.argv: return selftest()
    ap = argparse.ArgumentParser(); ap.add_argument('port'); ap.add_argument('baud', type=int); ap.add_argument('out')
    ap.add_argument('--minutes', type=float, default=0, help='duracion de la captura (0 = hasta Ctrl-C)')
    ap.add_argument('--wait', type=float, default=10); ap.add_argument('--send', default='w')
    a = ap.parse_args()
    import serial
    ser = serial.Serial(a.port, a.baud, timeout=0.05)
    try: ser.set_buffer_size(rx_size=4 << 20)
    except Exception: pass
    print('ESPERA %.0f s para que se estabilice (no muevas las IMUs)...' % a.wait); time.sleep(a.wait); ser.reset_input_buffer()
    ser.write(a.send.encode()); print('CAPTURA: enviado %r. Ctrl-C para parar%s' % (a.send, '' if not a.minutes else ' (o a los %.0f min)' % a.minutes))
    p = RawParser(); f = None; t0 = time.time(); tp = t0; n = 0
    def stop():
        ser.write(b'x'); time.sleep(0.5); ser.reset_input_buffer()
    try:
        while True:
            d = ser.read(max(1, ser.in_waiting))
            if d:
                for imu, flags, cnt, t_s, w, q, temp in p.feed(d):
                    if f is None:
                        f = open(a.out, 'w', buffering=1 << 20); f.write('# ' + (p.header or 'RAW (sin cabecera)') + '\n')
                        f.write('host_s,imu,flags,cnt,t_s,w0,w1,w2,w3,w4,w5,w6,w7,w8,q0,q1,q2,q3,temp_c\n')
                    f.write('%.4f,%d,%d,%d,%.6f,%s,%s,%s\n' % (time.time() - t0, imu, flags, cnt, t_s, ','.join(map(str, w)), ','.join('%.8g' % v for v in q),
                                                   'nan' if temp is None else '%.4f' % (25.0 + temp / 16.0)))
                    n += 1
            now = time.time()
            if now - tp >= 30:
                tp = now; print('%5.1f min  registros %d  CRC malos %d  huecos %s  tramas no leidas %s' % ((now - t0) / 60, p.good, p.bad, p.gaps or 0, p.lost or 0), flush=True)
            if a.minutes and now - t0 >= a.minutes * 60: break
    except KeyboardInterrupt:
        pass
    stop()
    if f: f.close()
    print('FIN: %d registros en %s (%.1f min), CRC malos %d, tramas no leidas %s' % (n, a.out, (time.time() - t0) / 60, p.bad, p.lost or 0))
main()
