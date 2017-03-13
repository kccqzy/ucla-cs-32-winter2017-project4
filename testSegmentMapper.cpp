#include "MapLoader.cpp"
#include "SegmentMapper.cpp"
#include <algorithm>
#include <cassert>

void testSeg(SegmentMapper const& sm, std::string const& lat, std::string const& lon, std::string const& streetName) {
    GeoCoord here(lat, lon);
    auto r = sm.getSegments(here);
    assert(!r.empty());
    fprintf(stderr, "DEBUG: found %zu street segment(s) matching (%s,%s)\n", r.size(), lat.c_str(), lon.c_str());
    assert(std::any_of(r.begin(), r.end(), [&streetName](auto s) { return s.streetName == streetName; }));
}

int main() {
    MapLoader ml;
    bool r = ml.load("mapdata.txt");
    assert(r);
    SegmentMapper sm;
    sm.init(ml);

    testSeg(sm, "34.0544590", "-118.4801137", "10th Helena Drive");
    testSeg(sm, "34.0555267", "-118.4796954", "13th Helena Drive");
    testSeg(sm, "34.0555356", "-118.4798135", "13th Helena Drive");
    testSeg(sm, "34.0554131", "-118.4804510", "13th Helena Drive");
    testSeg(sm, "34.0610306", "-118.4473883", "Gayley Avenue");
    {
        GeoCoord here("123.7654321", "-321.1234567");
        assert(sm.getSegments(here).empty());
    }
}
