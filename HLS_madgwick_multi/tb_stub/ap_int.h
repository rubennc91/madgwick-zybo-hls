// Stub minimo SOLO para probar en el PC (g++); Vitis HLS usa sus propias cabeceras.
#pragma once
#include <stdint.h>
template<int N> struct ap_uint {
    uint64_t v;
    ap_uint() : v(0) {}
    ap_uint(unsigned long long x) : v(x & ((N >= 64) ? ~0ull : ((1ull << N) - 1))) {}
    unsigned to_uint() const { return (unsigned)v; }
    operator unsigned() const { return (unsigned)v; }
    ap_uint operator+(int b) const { return ap_uint(v + (unsigned long long)b); }
};
template<int N> struct ap_int {
    int64_t v;
    ap_int() : v(0) {}
    ap_int(long long x) { int64_t m = (1ll << (N - 1)); int64_t t = x & ((1ll << N) - 1); v = (t ^ m) - m; }
    operator int() const { return (int)v; }
};
