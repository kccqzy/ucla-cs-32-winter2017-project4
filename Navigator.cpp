#include "provided.h"
#include "support.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <queue>
#include <string>
#include <vector>

#include <map>

bool operator<(GeoCoord const& a, GeoCoord const& b) {
    return std::make_pair(a.latitude, a.longitude) < std::make_pair(b.latitude, b.longitude);
}
bool operator==(GeoCoord const& a, GeoCoord const& b) {
    return std::make_pair(a.latitude, a.longitude) == std::make_pair(b.latitude, b.longitude);
}

class NavigatorImpl {
private:
    SegmentMapper segmentMapper;
    AttractionMapper attractionMapper;

    auto getNeighbors(GeoCoord const& gc) const {
        std::vector<GeoCoord> rv;
        for (auto const& seg : segmentMapper.getSegments(gc)) {
            if (gc == seg.segment.start)
                rv.emplace_back(seg.segment.end);
            else if (gc == seg.segment.end)
                rv.emplace_back(seg.segment.start);
            else
                assert(false && "getNeighbors: the provided coord is not the beginning or end of a street segment");
        }
        return rv;
    }

    struct DiscoveredNode {
        GeoCoord parent;
        double travelledDistance;
        double optimisticEstimate;
        DiscoveredNode(GeoCoord const& parent, double travelledDistance, double optimisticEstimate)
          : parent(parent), travelledDistance(travelledDistance), optimisticEstimate(optimisticEstimate) {}
    };
    typedef std::map<GeoCoord, DiscoveredNode> NodeMap;
    struct RankedNode {
        double rank;
        NodeMap::iterator it;
        RankedNode(double rank, NodeMap::iterator it) : rank(rank), it(it) {}
        friend bool operator<(RankedNode const& a, RankedNode const& b) { return a.rank > b.rank; }
    };
    typedef std::priority_queue<RankedNode> NodeRanks;

    static bool isGeoCoordOnSegment(GeoCoord const& gc, StreetSegment const& ss) {
        return gc == ss.segment.start || gc == ss.segment.end;
    }

    void
    reconstructPath(GeoCoord const& startCoord, GeoCoord const& endCoord, StreetSegment const& startStreetSegment,
                    GeoCoord const& last, NodeMap const& discoveredNodes, std::vector<NavSegment>& directions) const {
        // Unimplemented.
        (void) directions;
        std::vector<GeoCoord> path;
        path.emplace_back(endCoord);
        GeoCoord here = last;
        path.emplace_back(here);
        while (1) {
            path.emplace_back(here);
            if (isGeoCoordOnSegment(here, startStreetSegment)) break;
            auto i = discoveredNodes.find(here);
            assert(i != discoveredNodes.end());
            here = i->second.parent;
        }
        path.emplace_back(startCoord);
        std::reverse(path.begin(), path.end());
        fprintf(stderr, "GeoGraphics[{Red, Thick, GeoPath[{ {%s,%s}", path.front().latitudeText.c_str(),
                path.front().longitudeText.c_str());
        for (auto i = std::next(path.begin()); i != path.end(); ++i)
            fprintf(stderr, ",{%s,%s}", i->latitudeText.c_str(), i->longitudeText.c_str());
        fprintf(stderr, "}]}]\n");
    }

    bool getInfoFromAttrName(std::string const& attr, GeoCoord& gc, StreetSegment& ss) const {
        if (!attractionMapper.getGeoCoord(attr, gc)) return false;
        auto segments = segmentMapper.getSegments(gc);
        assert(segments.size() == 1);
        ss = segments.front();
        return true;
    }

    static void insertInitialNodes(GeoCoord const& startCoord, GeoCoord const& endCoord, GeoCoord const& routeBegin,
                                   NodeMap& discoveredNodes, NodeRanks& nodeRanks) {
        double distance = distanceEarthKM(startCoord, routeBegin);
        double estimate = distance + distanceEarthKM(routeBegin, endCoord);
        nodeRanks.emplace(estimate,
                          discoveredNodes.emplace(routeBegin, DiscoveredNode(routeBegin, distance, estimate)).first);
    }

public:
    bool loadMapData(std::string const& mapFile) {
        MapLoader ml;
        if (ml.load(mapFile)) {
            segmentMapper.init(ml);
            attractionMapper.init(ml);
            return true;
        }
        return false;
    }

    NavResult navigate(std::string const& start, std::string const& end, std::vector<NavSegment>& directions) const {
        GeoCoord startCoord, endCoord;
        StreetSegment startStreetSegment, endStreetSegment;
        if (!getInfoFromAttrName(start, startCoord, startStreetSegment)) return NAV_BAD_SOURCE;
        if (!getInfoFromAttrName(end, endCoord, endStreetSegment)) return NAV_BAD_DESTINATION;

        if (startStreetSegment.segment.start.latitude == endStreetSegment.segment.start.latitude &&
            startStreetSegment.segment.start.longitude == endStreetSegment.segment.start.longitude &&
            startStreetSegment.segment.end.latitude == endStreetSegment.segment.end.latitude &&
            startStreetSegment.segment.end.longitude == endStreetSegment.segment.end.longitude) {
            // reconstructDirectPath();
            return NAV_SUCCESS;
        }

        NodeMap discoveredNodes;
        NodeRanks nodeRanks;
        insertInitialNodes(startCoord, endCoord, startStreetSegment.segment.start, discoveredNodes, nodeRanks);
        insertInitialNodes(startCoord, endCoord, startStreetSegment.segment.end, discoveredNodes, nodeRanks);

        while (!nodeRanks.empty()) {
            auto currentIt = nodeRanks.top().it;
            nodeRanks.pop();
            if (currentIt->second.optimisticEstimate == HUGE_VAL) {
                // The priority_queue might contain duplicates, and the later
                // ones might have already been evaluated.
                continue;
            }
            if (isGeoCoordOnSegment(currentIt->first, endStreetSegment)) {
                reconstructPath(startCoord, endCoord, startStreetSegment, currentIt->first, discoveredNodes,
                                directions);
                return NAV_SUCCESS;
            }

            // Use this as a marker for completion of evaluation of a node.
            currentIt->second.optimisticEstimate = HUGE_VAL;
            for (auto const& neighbor : getNeighbors(currentIt->first)) {
                double distance = currentIt->second.travelledDistance + distanceEarthKM(currentIt->first, neighbor);
                assert(distance > 0);
                double estimate = distance + distanceEarthKM(neighbor, endCoord);
                assert(estimate > 0);
                auto it = discoveredNodes.find(neighbor);
                if (it == discoveredNodes.end()) {
                    nodeRanks.emplace(
                      estimate,
                      discoveredNodes.emplace(neighbor, DiscoveredNode(currentIt->first, distance, estimate)).first);
                } else if (it->second.optimisticEstimate == HUGE_VAL || distance >= it->second.travelledDistance) {
                    continue;
                } else {
                    // Inserting duplicate. Statistics have shown that in
                    // practice only 8% of inserts are duplicates.
                    it->second = DiscoveredNode(currentIt->first, distance, estimate);
                    nodeRanks.emplace(estimate, it);
                }
            }
        }
        return NAV_NO_ROUTE;
    }
};

//******************** Navigator functions ************************************

// These functions simply delegate to NavigatorImpl's functions.

Navigator::Navigator() { m_impl = new NavigatorImpl; }

Navigator::~Navigator() { delete m_impl; }

bool Navigator::loadMapData(std::string mapFile) { return m_impl->loadMapData(mapFile); }

NavResult Navigator::navigate(std::string start, std::string end, std::vector<NavSegment>& directions) const {
    return m_impl->navigate(start, end, directions);
}
