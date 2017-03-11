#include "MyMap.h"
#include "provided.h"
#include <cassert>
#include <cctype>
#include <string>

class AttractionMapperImpl {
private:
    MyMap<std::string, GeoCoord> m_map;

public:
    void init(const MapLoader& ml) {
        m_map.clear();
        for (size_t i = 0, ie = ml.getNumSegments(); i < ie; ++i) {
            // Don't hide data. MapLoader should have been a simple function
            // that returns a vector. Then we can use range-for loops here
            // because vector provides it.
            StreetSegment seg;
            if (!ml.getSegment(i, seg)) assert(false && "cannot get valid segment index");
            for (auto const& attr : seg.attractions) {
                auto name = attr.name;
                for (auto& c : name) c = std::tolower(c);
                m_map.associate(name, attr.geocoordinates);
            }
        }
    }
    bool getGeoCoord(std::string name, GeoCoord& gc) const {
        for (auto& c : name) c = std::tolower(c);
        if (auto const* c = m_map.find(name)) {
            gc = *c;
            return true;
        }
        return false;
    }
};

//******************** AttractionMapper functions *****************************

// These functions simply delegate to AttractionMapperImpl's functions.

AttractionMapper::AttractionMapper() { m_impl = new AttractionMapperImpl; }

AttractionMapper::~AttractionMapper() { delete m_impl; }

void AttractionMapper::init(const MapLoader& ml) { m_impl->init(ml); }

bool AttractionMapper::getGeoCoord(std::string attraction, GeoCoord& gc) const {
    return m_impl->getGeoCoord(attraction, gc);
}
