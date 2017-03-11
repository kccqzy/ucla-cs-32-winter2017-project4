#include "provided.h"
#include <fstream>
#include <istream>
#include <string>
#include <vector>

class MapLoaderImpl {
private:
    std::vector<StreetSegment> m_segments;

public:
    size_t getNumSegments() const { return m_segments.size(); }
    bool getSegment(size_t segNum, StreetSegment& seg) const {
        if (segNum < m_segments.size()) {
            seg = m_segments[segNum];
            return true;
        }
        return false;
    }
    bool load(std::string const& mapFile) noexcept {
        std::ifstream fs(mapFile);
        if (!fs) return false;
        std::vector<StreetSegment> segments; // Swap at end

        try {
            while (1) {
                fs.exceptions(std::ifstream::badbit | std::ifstream::failbit);
                std::string streetName, begLat, begLon, endLat, endLon, attractionsCountStr;
                if (!std::getline(fs, streetName)) break;
                fs.exceptions(std::ifstream::badbit | std::ifstream::failbit | std::ifstream::eofbit);
                std::getline(fs, begLat, ',');
                std::getline(fs, begLon, ' ');
                std::getline(fs, endLat, ',');
                std::getline(fs, endLon);
                std::getline(fs, attractionsCountStr);
                size_t attractionsCount = std::stoll(attractionsCountStr);
                std::vector<Attraction> attractions;
                while (attractionsCount--) {
                    std::string attrName, attrLat, attrLon;
                    std::getline(fs, attrName, '|');
                    std::getline(fs, attrLat, ',');
                    std::getline(fs, attrLon);
                    attractions.push_back({std::move(attrName), {std::move(attrLat), std::move(attrLon)}});
                }
                segments.push_back({std::move(streetName),
                                    {{std::move(begLat), std::move(begLon)}, {std::move(endLat), std::move(endLon)}},
                                    std::move(attractions)});
            }
        } catch (...) { return false; }
        m_segments.clear();
        std::swap(segments, m_segments);
        return true;
    }
};

//******************** MapLoader functions ************************************

// These functions simply delegate to MapLoaderImpl's functions.

MapLoader::MapLoader() { m_impl = new MapLoaderImpl; }

MapLoader::~MapLoader() { delete m_impl; }

bool MapLoader::load(std::string mapFile) { return m_impl->load(mapFile); }

size_t MapLoader::getNumSegments() const { return m_impl->getNumSegments(); }

bool MapLoader::getSegment(size_t segNum, StreetSegment& seg) const { return m_impl->getSegment(segNum, seg); }
