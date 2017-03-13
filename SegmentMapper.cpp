#include "MyMap.h"
#include "provided.h"
#include <cassert>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>
#include <vector>

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

class SegmentMapperImpl {
private:
    MyMap<RawCoord, std::vector<StreetSegment>> m_map;

    void insertInto(GeoCoord const& gc, StreetSegment const& sg) {
        if (auto* segs = m_map.find(gc))
            segs->push_back(sg);
        else
            m_map.associate(gc, {sg});
    }

public:
    void init(const MapLoader& ml) {
        m_map.clear();
        for (size_t i = 0, ie = ml.getNumSegments(); i < ie; ++i) {
            StreetSegment seg;
            if (!ml.getSegment(i, seg)) assert(false && "cannot get valid segment index");
            insertInto(seg.segment.start, seg);
            insertInto(seg.segment.end, seg);
            for (auto const& attr : seg.attractions) insertInto(attr.geocoordinates, seg);
        }
    }
    std::vector<StreetSegment> getSegments(const GeoCoord& gc) const {
        if (auto const* segs = m_map.find(gc)) return *segs;
        return {};
    }
};

//******************** SegmentMapper functions ********************************

// These functions simply delegate to SegmentMapperImpl's functions.

SegmentMapper::SegmentMapper() { m_impl = new SegmentMapperImpl; }

SegmentMapper::~SegmentMapper() { delete m_impl; }

void SegmentMapper::init(const MapLoader& ml) { m_impl->init(ml); }

std::vector<StreetSegment> SegmentMapper::getSegments(const GeoCoord& gc) const { return m_impl->getSegments(gc); }
