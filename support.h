#ifndef SUPPORT_H_
#define SUPPORT_H_
#include "MyMap.h"
#include "provided.h"
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>

template<typename R, typename T>
typename std::enable_if<sizeof(R) == sizeof(T), R>::type reinterpret(T t) {
    R r;
    memcpy(&r, &t, sizeof(R));
    return r;
}

struct RawCoord : public std::pair<int64_t, int64_t> {
    RawCoord(double lat, double lon) : pair(reinterpret<int64_t>(lat), reinterpret<int64_t>(lon)) {}
    RawCoord(GeoCoord const& gc) : RawCoord(gc.latitude, gc.longitude) {}
};

template<typename V>
using CoordMap = MyMap<RawCoord, V>;

#endif
