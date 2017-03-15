#include "MyMap.h"
#include "provided.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <memory>
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
    typedef std::shared_ptr<std::vector<std::pair<GeoCoord, StreetName>>> Neighbors;
    MyMap<GeoCoord, Neighbors> neighborMap;
    Neighbors getNeighbors(GeoCoord const& gc) {
        if (Neighbors* neighbors = neighborMap.find(gc)) return *neighbors;
        auto rv = std::make_shared<std::vector<std::pair<GeoCoord, StreetName>>>();
        auto segments = segmentMapper.getSegments(gc);
        rv->reserve(segments.size());
        for (auto const& seg : segments) {
            if (gc == seg.segment.start)
                rv->emplace_back(seg.segment.end, seg.streetName);
            else if (gc == seg.segment.end)
                rv->emplace_back(seg.segment.start, seg.streetName);
            // A coordinate can be both the beginning or end of a street segment
            // as well as an attraction.
        }
        assert(!rv->empty() && "getNeighbors: the provided coord is not the beginning or end of a street segment");
        neighborMap.associate(gc, rv);
        return rv;
    }

    typedef std::shared_ptr<StreetName const> StreetNameRef;
    typedef std::shared_ptr<GeoCoord const> GeoCoordRefRaw;
    struct GeoCoordRef {
        GeoCoordRefRaw ref;
        GeoCoordRef(GeoCoordRefRaw const& p) : ref(p) {}
        operator GeoCoord const&() const { return *ref; }
        friend bool operator<(GeoCoordRef const& a, GeoCoordRef const& b) { return *a.ref < *b.ref; }
    };
    struct DiscoveredNode {
        GeoCoordRef parent;
        StreetNameRef streetName;
        double distance;
        double estimate;
        DiscoveredNode(GeoCoordRef const& parent, double distance, double estimate, StreetNameRef const& streetName)
          : parent(parent), streetName(streetName), distance(distance), estimate(estimate) {}
    };
    typedef MyMap<GeoCoordRef, DiscoveredNode> NodeMap;
    struct RankedNode {
        double rank;
        DiscoveredNode* it;
        GeoCoordRef coord;
        RankedNode(double rank, DiscoveredNode* it, GeoCoordRef const& coord) : rank(rank), it(it), coord(coord) {}
        friend bool operator<(RankedNode const& a, RankedNode const& b) { return a.rank > b.rank; }
    };
    typedef std::priority_queue<RankedNode> NodeRanks;

    static bool isGeoCoordOnSegment(GeoCoord const& gc, StreetSegment const& ss) {
        return gc == ss.segment.start || gc == ss.segment.end;
    }

    static std::string describeDirection(double bearingDeg) {
        switch (static_cast<int>(std::floor((bearingDeg + 22.5) / 45.0)) % 8) {
        case 0: return "east";
        case 1: return "northeast";
        case 2: return "north";
        case 3: return "northwest";
        case 4: return "west";
        case 5: return "southwest";
        case 6: return "south";
        case 7: return "southeast";
        default: assert(false);
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
                    StreetSegment const& endStreetSegment, GeoCoordRef here, NodeMap const& discoveredNodes,
                    std::vector<NavSegment>& directions) {
        directions.clear();
        StreetNameRef previousStreetName = std::make_shared<StreetName>(endStreetSegment.streetName);
        for (GeoCoordRef previous = std::make_shared<GeoCoord const>(endCoord);
             !isGeoCoordOnSegment(previous, startStreetSegment);) {
            auto i = discoveredNodes.find(here);
            assert(i);
            auto thisStreetName = i->streetName;
            directions.emplace_back(makeProceedSegment(here, previous, *previousStreetName));
            if (*thisStreetName != *previousStreetName) directions.emplace_back(std::string{}, *previousStreetName);
            previousStreetName = std::move(thisStreetName);
            GeoCoordRef previous2 = std::move(previous);
            previous = std::move(here);
            here = i->parent;
            if (directions.back().m_command == NavSegment::TURN)
                directions.back().m_direction = describeTurn(here, previous, previous2);
        }
        directions.emplace_back(makeProceedSegment(startCoord, here, startStreetSegment.streetName));
        std::reverse(directions.begin(), directions.end());
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

    static void insertInitialNodes(GeoCoord const& startCoord, GeoCoord const& endCoord, GeoCoordRef const& routeBegin,
                                   StreetName const& streetName, NodeMap& discoveredNodes, NodeRanks& nodeRanks) {
        double distance = distanceEarthKM(startCoord, routeBegin);
        double estimate = distance + distanceEarthKM(routeBegin, endCoord);
        discoveredNodes.associate(
          routeBegin, DiscoveredNode(routeBegin, distance, estimate, std::make_shared<StreetName>(streetName)));
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

    NavResult navigate(std::string const& start, std::string const& end, std::vector<NavSegment>& directions) {
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
        insertInitialNodes(startCoord, endCoord, std::make_shared<GeoCoord const>(startStreetSegment.segment.start),
                           startStreetSegment.streetName, discoveredNodes, nodeRanks);
        insertInitialNodes(startCoord, endCoord, std::make_shared<GeoCoord const>(startStreetSegment.segment.end),
                           startStreetSegment.streetName, discoveredNodes, nodeRanks);

        while (!nodeRanks.empty()) {
            auto currentIt = nodeRanks.top().it;
            auto currentCoord = nodeRanks.top().coord;
            nodeRanks.pop();
            if (currentIt->estimate == HUGE_VAL) continue;
            // The priority_queue might contain duplicates, and the later
            // ones might have already been evaluated.

            if (isGeoCoordOnSegment(currentCoord, endStreetSegment))
                // Found it. Success.
                return reconstructPath(startCoord, endCoord, startStreetSegment, endStreetSegment, currentCoord,
                                       discoveredNodes, directions);

            // Use this as a marker for completion of evaluation of a node.
            currentIt->estimate = HUGE_VAL;
            auto neighbors = getNeighbors(currentCoord);
            for (auto const& neighbor : *neighbors) {
                double distance = currentIt->distance + distanceEarthKM(currentCoord, neighbor.first);
                assert(distance > 0);
                double estimate = distance + distanceEarthKM(neighbor.first, endCoord);
                assert(estimate > 0);
                GeoCoordRef neighborCoord(GeoCoordRefRaw(neighbors, &neighbor.first));
                if (auto it = discoveredNodes.find(neighborCoord)) {
                    if (it->estimate == HUGE_VAL || distance >= it->distance) continue;
                    // Inserting duplicate. Statistics have shown that in
                    // practice only 8% of inserts are duplicates.
                    *it = DiscoveredNode(currentCoord, distance, estimate, StreetNameRef(neighbors, &neighbor.second));
                    nodeRanks.emplace(estimate, it, neighborCoord);
                } else {
                    discoveredNodes.associate(
                      neighborCoord,
                      DiscoveredNode(currentCoord, distance, estimate, StreetNameRef(neighbors, &neighbor.second)));
                    nodeRanks.emplace(estimate, discoveredNodes.find(neighborCoord), neighborCoord);
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
