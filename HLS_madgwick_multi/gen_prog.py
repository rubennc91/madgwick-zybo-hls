#!/usr/bin/env python3
"""Genera madgwick_prog.h: el filtro Madgwick como programa para el interprete HLS.

Traza el MISMO algoritmo y en el MISMO orden de operaciones que
HLS_madgwick_stream_opt/madgwick_stream.cpp.  Los signos negativos se propagan
simbolicamente: (-a)*b = -(a*b), x+(-y) = x-y, (-x)+(-y) = -(x+y); las tres
igualdades son exactas en IEEE-754, asi que no se pierde ni un bit.

Uso: python3 gen_prog.py [--exact-rsqrt] > madgwick_prog.h
  --exact-rsqrt : sustituye Newton-Raphson por operaciones RSQRT/SQRT exactas
                  (SOLO para verificar en el host que el interprete es bit-exacto).
"""
import struct, sys

def f32(x):
    return struct.unpack('<f', struct.pack('<f', x))[0]
def bits(x):
    return struct.unpack('<I', struct.pack('<f', x))[0]

OPS = {'ADD': 0, 'SUB': 1, 'MUL': 2, 'GT': 3, 'RSQINIT': 4, 'RSQRT': 5, 'SQRT': 6}
EXACT = '--exact-rsqrt' in sys.argv

# Entradas fijas (registros 0..14)
IN_G, IN_A, IN_M, IN_Q, IN_BETA, IN_DT = 0, 3, 6, 9, 13, 14
N_IN = 15
CONSTS = [0.5, 1.0, 1.5, 2.0, 4.0, 8.0, f32(1e-20), -0.5]
CID = {c: N_IN + i for i, c in enumerate(CONSTS)}
CBASE = N_IN
FIRST_FREE = N_IN + len(CONSTS)


class V:
    def __init__(s, p, i, neg=False):
        s.p, s.i, s.neg = p, i, neg
    def __neg__(s):
        return V(s.p, s.i, not s.neg)
    def _w(s, o):
        return o if isinstance(o, V) else s.p.const(o)
    def __mul__(s, o):
        o = s._w(o)
        return V(s.p, s.p.emit('MUL', s.i, o.i), s.neg ^ o.neg)
    __rmul__ = __mul__
    def __add__(s, o):
        o = s._w(o)
        if s.neg == o.neg:
            return V(s.p, s.p.emit('ADD', s.i, o.i), s.neg)
        if o.neg:                      # x + (-y)  = x - y
            return V(s.p, s.p.emit('SUB', s.i, o.i), False)
        return V(s.p, s.p.emit('SUB', o.i, s.i), False)   # (-x) + y = y - x
    def __radd__(s, o):
        return s._w(o) + s
    def __sub__(s, o):
        return s + (-s._w(o))
    def __rsub__(s, o):
        return s._w(o) + (-s)


class Prog:
    def __init__(s):
        s.ins = []           # (op, a, b)
        s.cse = {}
    def const(s, c):
        c = f32(c)
        if c not in CID:
            raise KeyError('constante no registrada: %r' % c)
        return V(s, CID[c])
    def emit(s, op, a, b):
        key = (op, min(a, b), max(a, b)) if op in ('ADD', 'MUL') else (op, a, b)
        if key in s.cse:
            return s.cse[key]
        vid = 10000 + len(s.ins)
        s.ins.append((op, a, b, vid))
        s.cse[key] = vid
        return vid
    def need_pos(s, x, what):
        if x.neg:
            raise ValueError('valor negado simbolicamente en %s' % what)
        return x

    # ---- primitivas ----
    def rsqrt(s, x):
        s.need_pos(x, 'rsqrt')
        if EXACT:
            return V(s, s.emit('RSQRT', x.i, x.i))
        y = V(s, s.emit('RSQINIT', x.i, x.i))
        hx = x * 0.5
        for _ in range(3):
            y2 = y * y
            t = hx * y2
            u = 1.5 - t
            y = y * u
        return y
    def sqrt(s, x):
        s.need_pos(x, 'sqrt')
        if EXACT:
            return V(s, s.emit('SQRT', x.i, x.i))
        return x * s.rsqrt(x)        # x==0 -> 0*finito = 0
    def gt(s, x, c):
        s.need_pos(x, 'gt')
        return V(s, s.emit('GT', x.i, s.const(c).i))


def inputs(p):
    g = [V(p, IN_G + k) for k in range(3)]
    a = [V(p, IN_A + k) for k in range(3)]
    m = [V(p, IN_M + k) for k in range(3)]
    q = [V(p, IN_Q + k) for k in range(4)]
    return g, a, m, q, V(p, IN_BETA), V(p, IN_DT)


def qdot(q, g):
    q0, q1, q2, q3 = q
    gx, gy, gz = g
    d1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
    d2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
    d3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
    d4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)
    return d1, d2, d3, d4


def normalize_s(p, s0, s1, s2, s3):
    n2 = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3
    f = p.gt(n2, 1e-20)                       # 1.0 o 0.0
    r = p.rsqrt(n2)
    r = r * f + (1.0 - f)                     # f=0 -> 1.0 exacto: s no cambia
    return s0 * r, s1 * r, s2 * r, s3 * r


def tail(p, q, d, beta_s, dt):
    q0, q1, q2, q3 = q
    d1, d2, d3, d4 = d
    if beta_s is not None:
        s0, s1, s2, s3 = beta_s
    q0 = q0 + d1 * dt
    q1 = q1 + d2 * dt
    q2 = q2 + d3 * dt
    q3 = q3 + d4 * dt
    r = p.rsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
    return [q0 * r, q1 * r, q2 * r, q3 * r]


def prog_gyro():
    p = Prog()
    g, a, m, q, beta, dt = inputs(p)
    d = qdot(q, g)
    return p, tail(p, q, d, None, dt)


def prog_imu():
    p = Prog()
    g, a, m, q, beta, dt = inputs(p)
    q0, q1, q2, q3 = q
    ax, ay, az = a
    d1, d2, d3, d4 = qdot(q, g)
    r = p.rsqrt(ax * ax + ay * ay + az * az)
    ax, ay, az = ax * r, ay * r, az * r
    _2q0 = 2.0 * q0; _2q1 = 2.0 * q1; _2q2 = 2.0 * q2; _2q3 = 2.0 * q3
    _4q0 = 4.0 * q0; _4q1 = 4.0 * q1; _4q2 = 4.0 * q2
    _8q1 = 8.0 * q1; _8q2 = 8.0 * q2
    q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
    s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
    s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay
    s0, s1, s2, s3 = normalize_s(p, s0, s1, s2, s3)
    d1 = d1 - beta * s0
    d2 = d2 - beta * s1
    d3 = d3 - beta * s2
    d4 = d4 - beta * s3
    return p, tail(p, q, (d1, d2, d3, d4), None, dt)


def prog_ahrs():
    p = Prog()
    g, a, m, q, beta, dt = inputs(p)
    q0, q1, q2, q3 = q
    ax, ay, az = a
    mx, my, mz = m
    d1, d2, d3, d4 = qdot(q, g)
    r = p.rsqrt(ax * ax + ay * ay + az * az)
    ax, ay, az = ax * r, ay * r, az * r
    r = p.rsqrt(mx * mx + my * my + mz * mz)
    mx, my, mz = mx * r, my * r, mz * r

    _2q0mx = 2.0 * q0 * mx
    _2q0my = 2.0 * q0 * my
    _2q0mz = 2.0 * q0 * mz
    _2q1mx = 2.0 * q1 * mx
    _2q0 = 2.0 * q0; _2q1 = 2.0 * q1; _2q2 = 2.0 * q2; _2q3 = 2.0 * q3
    _2q0q2 = 2.0 * q0 * q2
    _2q2q3 = 2.0 * q2 * q3
    q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3
    q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3
    q2q2 = q2 * q2; q2q3 = q2 * q3; q3q3 = q3 * q3

    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3
    _2bx = p.sqrt(hx * hx + hy * hy)
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3
    _4bx = 2.0 * _2bx
    _4bz = 2.0 * _2bz

    eax = 2.0 * q1q3 - _2q0q2 - ax
    eay = 2.0 * q0q1 + _2q2q3 - ay
    eaz = 1.0 - 2.0 * q1q1 - 2.0 * q2q2 - az
    emx = _2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx
    emy = _2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my
    emz = _2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz

    s0 = -_2q2 * eax + _2q1 * eay - _2bz * q2 * emx + (-_2bx * q3 + _2bz * q1) * emy + _2bx * q2 * emz
    s1 = _2q3 * eax + _2q0 * eay - 4.0 * q1 * eaz + _2bz * q3 * emx + (_2bx * q2 + _2bz * q0) * emy + (_2bx * q3 - _4bz * q1) * emz
    s2 = -_2q0 * eax + _2q3 * eay - 4.0 * q2 * eaz + (-_4bx * q2 - _2bz * q0) * emx + (_2bx * q1 + _2bz * q3) * emy + (_2bx * q0 - _4bz * q2) * emz
    s3 = _2q1 * eax + _2q2 * eay + (-_4bx * q3 + _2bz * q1) * emx + (-_2bx * q0 + _2bz * q2) * emy + _2bx * q1 * emz

    s0, s1, s2, s3 = normalize_s(p, s0, s1, s2, s3)
    d1 = d1 - beta * s0
    d2 = d2 - beta * s1
    d3 = d3 - beta * s2
    d4 = d4 - beta * s3
    return p, tail(p, q, (d1, d2, d3, d4), None, dt)


def allocate(p, outs):
    """Asignacion lineal de registros. Devuelve (instrucciones con regs, regs de salida, nregs)."""
    for o in outs:
        if o.neg:
            raise ValueError('salida negada')
    out_ids = [o.i for o in outs]
    last = {}
    for k, (op, a, b, vid) in enumerate(p.ins):
        last[a] = k
        last[b] = k
    for oid in out_ids:
        last[oid] = len(p.ins)           # vivas hasta el final
    reg = {i: i for i in range(N_IN + len(CONSTS))}
    free = []
    nxt = FIRST_FREE
    pinned = set(range(N_IN, N_IN + len(CONSTS)))
    # entradas muertas antes de empezar (no usadas) -> libres
    used = set()
    for (op, a, b, vid) in p.ins:
        used.add(a); used.add(b)
    for i in range(N_IN):
        if i not in used:
            free.append(i)
    code = []
    for k, (op, a, b, vid) in enumerate(p.ins):
        ra, rb = reg[a], reg[b]
        # liberar operandos que mueren aqui (se pueden reutilizar como destino)
        for v in {a, b}:
            if last.get(v) == k and reg[v] not in pinned and v not in out_ids:
                free.append(reg[v])
        if free:
            rd = free.pop(0)
        else:
            rd = nxt; nxt += 1
        reg[vid] = rd
        code.append((OPS[op], rd, ra, rb))
        # si el propio resultado nunca se usa (no deberia pasar)
    return code, [reg[i] for i in out_ids], nxt


def encode(c):
    op, d, a, b = c
    return op | (d << 4) | (a << 11) | (b << 18)


def main():
    progs = [('AHRS', prog_ahrs), ('IMU', prog_imu), ('GYRO', prog_gyro)]
    rom = []; starts = []; ends = []; outs_regs = []; nreg = 0
    stats = []
    for name, fn in progs:
        p, outs = fn()
        code, orr, nr = allocate(p, outs)
        starts.append(len(rom)); rom.extend(encode(c) for c in code); ends.append(len(rom))
        outs_regs.append(orr); nreg = max(nreg, nr)
        stats.append((name, len(code), nr))
    if nreg > 128:
        raise SystemExit('demasiados registros: %d' % nreg)
    w = sys.stdout.write
    w('// GENERADO por gen_prog.py%s -- no editar a mano.\n' % (' --exact-rsqrt (SOLO TEST)' if EXACT else ''))
    for s in stats:
        w('//   programa %-5s: %3d instrucciones, %3d registros\n' % s)
    w('#ifndef MADGWICK_PROG_H\n#define MADGWICK_PROG_H\n\n')
    w('#define MP_NREG   %d\n' % (1 << max(6, (nreg - 1).bit_length())))
    w('#define MP_NIN    %d   /* g(3) a(3) m(3) q(4) beta dt */\n' % N_IN)
    w('#define MP_NCONST %d\n' % len(CONSTS))
    w('#define MP_CBASE  %d\n' % CBASE)
    w('#define MP_ROMLEN %d\n' % len(rom))
    w('#define OP_ADD %d\n#define OP_SUB %d\n#define OP_MUL %d\n#define OP_GT %d\n#define OP_RSQINIT %d\n#define OP_RSQRT %d\n#define OP_SQRT %d\n' %
      tuple(OPS[k] for k in ['ADD', 'SUB', 'MUL', 'GT', 'RSQINIT', 'RSQRT', 'SQRT']))
    w('enum { MP_AHRS = 0, MP_IMU = 1, MP_GYRO = 2 };\n\n')
    w('static const unsigned MP_CONST_BITS[MP_NCONST] = {%s};\n' % ', '.join('0x%08xu' % bits(c) for c in CONSTS))
    w('static const unsigned short MP_START[3] = {%s};\n' % ', '.join(map(str, starts)))
    w('static const unsigned short MP_END[3]   = {%s};\n' % ', '.join(map(str, ends)))
    w('static const unsigned char  MP_OUT[3][4] = {%s};\n' %
      ', '.join('{%s}' % ', '.join(map(str, o)) for o in outs_regs))
    w('static const unsigned MP_ROM[MP_ROMLEN] = {\n')
    for i in range(0, len(rom), 8):
        w('    ' + ', '.join('0x%07xu' % x for x in rom[i:i + 8]) + ',\n')
    w('};\n\n#endif\n')

main()
