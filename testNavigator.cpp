#include "Navigator.cpp"
#include "SegmentMapper.cpp"
#include "AttractionMapper.cpp"
#include "MapLoader.cpp"
#include <cassert>

void testNav(Navigator const& nav, std::string begin, std::string end) {
            std::vector<NavSegment> directions;
        NavResult nr = nav.navigate(begin,end, directions);
        fprintf(stderr, "result = %d\n", nr);
        assert(nr == NAV_SUCCESS);

}

int main() {
    Navigator nav;
    bool r = nav.loadMapData("mapdata.txt");
    assert(r);
    testNav(nav, "1061 Broxton Avenue", "Headlines!");
    testNav(nav, "UCLA Opus Project", "UCLA Wilshire Center");
    testNav(nav, "Covel Commons", "Intramural Field");
    // testNav(nav, "Brentwood Country Mart", "Saint Sebastian School");
}
