#include "MyMap.h"
#include "provided.h"
#include <cassert>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>
#include <vector>

namespace {
template<typename R, typename T, typename = typename std::enable_if<sizeof(R) == sizeof(T)>::type>
R reinterpret(T t) {
    R r;
    memcpy(&r, &t, sizeof(R));
    return r;
}
}

class SegmentMapperImpl {
private:
    std::vector<StreetSegment> m_segments;
    MyMap<std::pair<int64_t, int64_t>, std::vector<StreetSegment const*>> m_map;

    void insertInto(GeoCoord const& gc, StreetSegment const* sg) {
        auto key = std::make_pair(reinterpret<int64_t>(gc.latitude), reinterpret<int64_t>(gc.longitude));
        if (auto* segs = m_map.find(key))
            segs->push_back(sg);
        else
            m_map.associate(key, {sg});
    }

public:
    void init(const MapLoader& ml) {
        m_map.clear();
        m_segments.clear();
        m_segments.reserve(ml.getNumSegments());
        for (size_t i = 0, ie = ml.getNumSegments(); i < ie; ++i) {
            StreetSegment seg;
            if (!ml.getSegment(i, seg)) assert(false && "cannot get valid segment index");
            m_segments.emplace_back(std::move(seg));
        }
        for (auto const& seg : m_segments) {
            insertInto(seg.segment.start, &seg);
            insertInto(seg.segment.end, &seg);
            for (auto const& attr : seg.attractions) insertInto(attr.geocoordinates, &seg);
        }
    }
    auto getSegments(const GeoCoord& gc) const {
        std::vector<StreetSegment> segments;
        if (auto const* segs = m_map.find(std::make_pair(reinterpret<int64_t>(gc.latitude), reinterpret<int64_t>(gc.longitude))))
            for (auto const* seg : *segs) segments.emplace_back(*seg);
        return segments;
    }
};

//******************** SegmentMapper functions ********************************

// These functions simply delegate to SegmentMapperImpl's functions.

SegmentMapper::SegmentMapper() { m_impl = new SegmentMapperImpl; }

SegmentMapper::~SegmentMapper() { delete m_impl; }

void SegmentMapper::init(const MapLoader& ml) { m_impl->init(ml); }

std::vector<StreetSegment> SegmentMapper::getSegments(const GeoCoord& gc) const { return m_impl->getSegments(gc); }
