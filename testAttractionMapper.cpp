#include "AttractionMapper.cpp"
#include "MapLoader.cpp"
#include <cassert>

void testAttr(AttractionMapper const& am, std::string const& attr, std::string const& lat, std::string const& lon) {
            GeoCoord gc;
        GeoCoord actual(lat,lon);
        bool found = am.getGeoCoord(attr, gc);
        assert(found);
        assert(gc.latitude == actual.latitude);
        assert(gc.longitude == actual.longitude);
        assert(gc.latitudeText == actual.latitudeText);
        assert(gc.longitudeText == actual.longitudeText);

}

int main() {
    MapLoader ml;
    bool r = ml.load("mapdata.txt");
    assert(r);
    AttractionMapper am;
    am.init(ml);

    testAttr(am, "Saint Sebastian School", "34.0464796", "-118.4554219");
    testAttr(am, "West Los Angeles Finance Station Los Angeles Post Office", "34.0455535", "-118.4513871");
    testAttr(am, "Ruth Swissa Permanent Makeup and Skin;Bier Beisl", "34.0688725", "-118.4068761");
    testAttr(am, "Veterans Administration Medical Center West Los Angeles Heliport", "34.0498465", "-118.4562274");
    testAttr(am, "veterans administration medical center west los angeles heliport", "34.0498465", "-118.4562274");
    testAttr(am, "VETERANS ADMINISTRATION MEDICAL CENTER WEST LOS ANGELES HELIPORT", "34.0498465", "-118.4562274");
    testAttr(am, "VETERANS administration medical CENTER west LOS angeles HELIPORT", "34.0498465", "-118.4562274");
}
