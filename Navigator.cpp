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

    typedef std::string StreetName;
    auto getNeighbors(GeoCoord const& gc) const {
        std::vector<std::pair<GeoCoord, StreetName>> rv;
        for (auto const& seg : segmentMapper.getSegments(gc)) {
            if (gc == seg.segment.start)
                rv.emplace_back(seg.segment.end, seg.streetName);
            else if (gc == seg.segment.end)
                rv.emplace_back(seg.segment.start, seg.streetName);
            // A coordinate can be both the beginning or end of a street segment
            // as well as an attraction.
        }
        assert(!rv.empty() && "getNeighbors: the provided coord is not the beginning or end of a street segment");
        return rv;
    }

    struct DiscoveredNode {
        GeoCoord parent;
        StreetName streetName;
        double distance;
        double estimate;
        DiscoveredNode(GeoCoord const& parent, double distance, double estimate, StreetName const& streetName)
          : parent(parent), streetName(streetName), distance(distance), estimate(estimate) {}
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

    static std::string describeDirection(double bearingDeg) {
        int direction = std::floor((bearingDeg + 22.5) / 45.0);
        switch (direction) {
        case 0: return "east";
        case 1: return "northeast";
        case 2: return "north";
        case 3: return "northwest";
        case 4: return "west";
        case 5: return "southwest";
        case 6: return "south";
        case 7: return "southeast";
        default: assert(false && "invalid bearing when describing direction");
        }
    }

    static std::string describeTurn(GeoCoord const& from, GeoCoord const& via, GeoCoord const& to) {
        return angleBetween2Lines({from, via}, {via, to}) < 180.0 ? "left" : "right";
    }

    static NavSegment makeProceedSegment(GeoCoord const& from, GeoCoord const& to, StreetName const& streetName) {
        GeoSegment gs(from, to);
        return {describeDirection(angleOfLine(gs)), streetName, distanceEarthMiles(from, to), gs};
    }

    static NavResult
    reconstructPath(GeoCoord const& startCoord, GeoCoord const& endCoord, StreetSegment const& startStreetSegment,
                    StreetSegment const& endStreetSegment, GeoCoord here, NodeMap const& discoveredNodes,
                    std::vector<NavSegment>& directions) {
        directions.clear();
        StreetName previousStreetName = endStreetSegment.streetName;
        for (GeoCoord previous = endCoord; !isGeoCoordOnSegment(previous, startStreetSegment);) {
            auto i = discoveredNodes.find(here);
            assert(i);
            auto thisStreetName = i->streetName;
            directions.emplace_back(makeProceedSegment(here, previous, previousStreetName));
            if (thisStreetName != previousStreetName) directions.emplace_back(std::string{}, previousStreetName);
            previousStreetName = std::move(thisStreetName);
            GeoCoord previous2 = std::move(previous);
            previous = std::move(here);
            here = i->parent;
            if (directions.back().m_command == NavSegment::TURN)
                directions.back().m_direction = describeTurn(here, previous, previous2);
        }
        directions.emplace_back(makeProceedSegment(startCoord, here, startStreetSegment.streetName));
        std::reverse(directions.begin(), directions.end());
        fprintf(stderr, "Successfully constructed turn-by-turn navigation with %zu steps\n", directions.size());
        return NAV_SUCCESS;
    }

    static NavResult reconstructDirectPath(GeoCoord const& startCoord, GeoCoord const& endCoord,
                                           StreetSegment const& streetSegment, std::vector<NavSegment>& directions) {
        directions.clear();
        directions.emplace_back(makeProceedSegment(startCoord, endCoord, streetSegment.streetName));
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
                                   StreetName const& streetName, NodeMap& discoveredNodes, NodeRanks& nodeRanks) {
        double distance = distanceEarthKM(startCoord, routeBegin);
        double estimate = distance + distanceEarthKM(routeBegin, endCoord);
        discoveredNodes.associate(routeBegin, DiscoveredNode(routeBegin, distance, estimate, streetName));
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
            startStreetSegment.segment.end.longitude == endStreetSegment.segment.end.longitude)
            return reconstructDirectPath(startCoord, endCoord, startStreetSegment, directions);


        NodeMap discoveredNodes;
        NodeRanks nodeRanks;
        insertInitialNodes(startCoord, endCoord, startStreetSegment.segment.start, startStreetSegment.streetName,
                           discoveredNodes, nodeRanks);
        insertInitialNodes(startCoord, endCoord, startStreetSegment.segment.end, startStreetSegment.streetName,
                           discoveredNodes, nodeRanks);

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
                return reconstructPath(startCoord, endCoord, startStreetSegment, endStreetSegment, currentCoord,
                                       discoveredNodes, directions);

            // Use this as a marker for completion of evaluation of a node.
            currentIt->estimate = HUGE_VAL;
            for (auto const& neighbor : getNeighbors(currentCoord)) {
                double distance = currentIt->distance + distanceEarthKM(currentCoord, neighbor.first);
                assert(distance > 0);
                double estimate = distance + distanceEarthKM(neighbor.first, endCoord);
                assert(estimate > 0);
                if (auto it = discoveredNodes.find(neighbor.first)) {
                    if (it->estimate == HUGE_VAL || distance >= it->distance) continue;
                    // Inserting duplicate. Statistics have shown that in
                    // practice only 8% of inserts are duplicates.
                    *it = DiscoveredNode(currentCoord, distance, estimate, neighbor.second);
                    nodeRanks.emplace(estimate, it, neighbor.first);
                } else {
                    discoveredNodes.associate(neighbor.first,
                                              DiscoveredNode(currentCoord, distance, estimate, neighbor.second));
                    nodeRanks.emplace(estimate, discoveredNodes.find(neighbor.first), neighbor.first);
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
