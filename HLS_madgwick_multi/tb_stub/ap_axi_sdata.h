#pragma once
#include "ap_int.h"
namespace hls {
template<class T, int U, int I, int D> struct axis {
    T data; int keep; int strb; unsigned user; int last; unsigned id; unsigned dest;
    axis() : data(), keep(0), strb(0), user(0), last(0), id(0), dest(0) {}
};
}
