#include "MyMap.h"
#include "provided.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <queue>
#include <string>
#include <vector>

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
            // A coordinate can be both the beginning or end of a street segment
            // as well as an attraction.
        }
        assert(!rv.empty() && "getNeighbors: the provided coord is not the beginning or end of a street segment");
        return rv;
    }

    struct DiscoveredNode {
        GeoCoord parent;
        double distance;
        double estimate;
        DiscoveredNode(GeoCoord const& parent, double distance, double estimate)
          : parent(parent), distance(distance), estimate(estimate) {}
    };
    typedef MyMap<GeoCoord, DiscoveredNode> NodeMap;
    struct RankedNode {
        double rank;
        DiscoveredNode* it;
        GeoCoord coord;
        RankedNode(double rank, DiscoveredNode* it, GeoCoord coord) : rank(rank), it(it), coord(coord) {}
        friend bool operator<(RankedNode const& a, RankedNode const& b) { return a.rank > b.rank; }
    };
    typedef std::priority_queue<RankedNode> NodeRanks;

    static bool isGeoCoordOnSegment(GeoCoord const& gc, StreetSegment const& ss) {
        return gc == ss.segment.start || gc == ss.segment.end;
    }

    NavResult
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
            assert(i);
            here = i->parent;
        }
        path.emplace_back(startCoord);
        std::reverse(path.begin(), path.end());
        fprintf(stderr, "GeoGraphics[{Red, Thick, GeoPath[{ {%s,%s}", path.front().latitudeText.c_str(),
                path.front().longitudeText.c_str());
        for (auto i = std::next(path.begin()); i != path.end(); ++i)
            fprintf(stderr, ",{%s,%s}", i->latitudeText.c_str(), i->longitudeText.c_str());
        fprintf(stderr, "}]}]\n");
        return NAV_SUCCESS;
    }

    bool getInfoFromAttrName(std::string const& attr, GeoCoord& gc, StreetSegment& ss) const {
        if (!attractionMapper.getGeoCoord(attr, gc)) return false;
        auto segments = segmentMapper.getSegments(gc);
        assert(!segments.empty());
        // There might legitimately be multiple street segments here due to a
        // coordinate might be both the beginning or end of a street segment as
        // well as an attraction.
        if (segments.size() == 1)
            ss = segments.front();
        else {
            auto p = std::find_if(segments.cbegin(), segments.cend(), [&attr](StreetSegment const& thisStreet) {
                return std::any_of(thisStreet.attractions.cbegin(), thisStreet.attractions.cend(),
                                   [&attr](Attraction const& thisAttr) { return thisAttr.name == attr; });
            });
            assert(p != segments.cend());
            ss = *p;
        }
        return true;
    }

    static void insertInitialNodes(GeoCoord const& startCoord, GeoCoord const& endCoord, GeoCoord const& routeBegin,
                                   NodeMap& discoveredNodes, NodeRanks& nodeRanks) {
        double distance = distanceEarthKM(startCoord, routeBegin);
        double estimate = distance + distanceEarthKM(routeBegin, endCoord);
        discoveredNodes.associate(routeBegin, DiscoveredNode(routeBegin, distance, estimate));
        nodeRanks.emplace(estimate, discoveredNodes.find(routeBegin), routeBegin);
    }

public:
    bool loadMapData(std::string const& mapFile) {
        MapLoader ml;
        if (!ml.load(mapFile)) return false;
        segmentMapper.init(ml);
        attractionMapper.init(ml);
        return true;
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
            fprintf(stderr, "In this iteration, there are %d nodes in the tree and %zu nodes in the priority_queue.\n",
                    discoveredNodes.size(), nodeRanks.size());
            auto currentIt = nodeRanks.top().it;
            auto currentCoord = nodeRanks.top().coord;
            fprintf(stderr,
                    "Receiving new node {%s,%s} from priority_queue with saved estimate = %.5f, distance = %.5f and "
                    "actual estimate = %.5f\n",
                    currentCoord.latitudeText.c_str(), currentCoord.longitudeText.c_str(), nodeRanks.top().rank,
                    currentIt->distance, currentIt->estimate);
            nodeRanks.pop();
            if (currentIt->estimate == HUGE_VAL) continue;
            // The priority_queue might contain duplicates, and the later
            // ones might have already been evaluated.
            fprintf(stderr,
                    "Investigating new node {%s,%s} from priority_queue with distance = %.5f and "
                    "actual estimate = %.5f\n",
                    currentCoord.latitudeText.c_str(), currentCoord.longitudeText.c_str(), currentIt->distance,
                    currentIt->estimate);

            if (isGeoCoordOnSegment(currentCoord, endStreetSegment))
                // Found it. Success.
                return reconstructPath(startCoord, endCoord, startStreetSegment, currentCoord, discoveredNodes,
                                       directions);

            // Use this as a marker for completion of evaluation of a node.
            currentIt->estimate = HUGE_VAL;
            for (auto const& neighbor : getNeighbors(currentCoord)) {
                double distance = currentIt->distance + distanceEarthKM(currentCoord, neighbor);
                assert(distance > 0);
                double estimate = distance + distanceEarthKM(neighbor, endCoord);
                assert(estimate > 0);
                if (auto it = discoveredNodes.find(neighbor)) {
                    if (it->estimate == HUGE_VAL || distance >= it->distance) continue;
                    // Inserting duplicate. Statistics have shown that in
                    // practice only 8% of inserts are duplicates.
                    *it = DiscoveredNode(currentCoord, distance, estimate);
                    nodeRanks.emplace(estimate, it, neighbor);
                } else {
                    discoveredNodes.associate(neighbor, DiscoveredNode(currentCoord, distance, estimate));
                    nodeRanks.emplace(estimate, discoveredNodes.find(neighbor), neighbor);
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
