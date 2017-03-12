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
                std::string streetName, begLat, begLon, endLat, endLon, attractionsCountStr;
                if (!std::getline(fs, streetName)) break;
                if (!std::getline(fs, begLat, ',')) return false;
                if (!std::getline(fs >> std::ws, begLon, ' ')) return false;
                if (!std::getline(fs, endLat, ',')) return false;
                if (!std::getline(fs >> std::ws, endLon)) return false;
                if (!std::getline(fs, attractionsCountStr)) return false;
                size_t attractionsCount = std::stoll(attractionsCountStr);
                std::vector<Attraction> attractions;
                for (size_t j = 0; j < attractionsCount; ++j) {
                    std::string attrName, attrLat, attrLon;
                    if (!std::getline(fs, attrName, '|')) return false;
                    if (!std::getline(fs, attrLat, ',')) return false;
                    if (!std::getline(fs >> std::ws, attrLon)) return false;
                    attractions.push_back({std::move(attrName), {std::move(attrLat), std::move(attrLon)}});
                }
                segments.push_back({std::move(streetName),
                                    {{std::move(begLat), std::move(begLon)}, {std::move(endLat), std::move(endLon)}},
                                    std::move(attractions)});
            }
        } catch (std::exception&) { return false; }
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
