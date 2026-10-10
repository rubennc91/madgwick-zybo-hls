#pragma once
#include <deque>
#include <cassert>
namespace hls {
template<class T> struct stream {
    std::deque<T> q;
    void write(const T& x) { q.push_back(x); }
    T read() { assert(!q.empty()); T x = q.front(); q.pop_front(); return x; }
    bool read_nb(T& x) { if (q.empty()) return false; x = q.front(); q.pop_front(); return true; }
    bool empty() const { return q.empty(); }
};
}
