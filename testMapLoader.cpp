#include "MapLoader.cpp"
#include <cassert>
#include <iostream>

int main() {
    {
        system("head -n 6 mapdata.txt > mapdata-simple.txt");
        MapLoader ml;
        bool r = ml.load("mapdata-simple.txt");
        assert(r);
        assert(ml.getNumSegments() == 2);
    }
    {
        MapLoader ml;
        bool r = ml.load("mapdata.txt");
        assert(r);
        assert(ml.getNumSegments() == 19641);
        FILE* nf = fopen("mapdata-new.txt", "w");
        for (size_t i = 0, ie = ml.getNumSegments(); i < ie; ++i) {
            StreetSegment seg;
            assert(ml.getSegment(i, seg));
            fprintf(nf, "%s\n%.7f, %.7f %.7f,%.7f\n%zu\n", seg.streetName.c_str(), seg.segment.start.latitude,
                    seg.segment.start.longitude, seg.segment.end.latitude, seg.segment.end.longitude,
                    seg.attractions.size());
            for (auto const& attr : seg.attractions) {
                fprintf(nf, "%s|%.7f, %.7f\n", attr.name.c_str(), attr.geocoordinates.latitude,
                        attr.geocoordinates.longitude);
            }
        }
        fclose(nf);
        int diff = system("diff -u mapdata.txt mapdata-new.txt");
        assert(diff == 0);
    }
}
