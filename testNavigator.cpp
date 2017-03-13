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
    for (volatile int i = 0; i < 200; ++i) {
        testNav(nav, "1061 Broxton Avenue", "Headlines!");
        testNav(nav, "UCLA Opus Project", "UCLA Wilshire Center");
        testNav(nav, "Intramural Field", "Covel Commons");
        testNav(nav, "David Geffen School of Medicine", "Covel Commons");
        testNav(nav, "David Geffen School of Medicine", "Twentieth Century Fox Film Corporation");
        testNav(nav, "Literati Cafe", "Residence Inn by Marriott Beverly Hills");
        testNav(nav, "Fresh corn grill ", "Saint Sebastian School");
    }
}
