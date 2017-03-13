#include "provided.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include <map>

bool operator<(GeoCoord const& a, GeoCoord const& b) {
    return std::make_pair(a.latitude, a.longitude) < std::make_pair(b.latitude, b.longitude);
}
bool operator==(GeoCoord const& a, GeoCoord const& b) {
    return std::make_pair(a.latitude, a.longitude) == std::make_pair(b.latitude, b.longitude);
}

namespace {
template<typename T, typename Comp>
class PairingHeap {
private:
    struct Node {
        T d;
        Node *youngestChild, *olderSibling, *parentOrYoungerSibling;
    };
    Comp comp;
    Node* root;
    Node* link(Node* a, Node* b) const {
        if (!a && !b) return nullptr;
        if (!a) return b;
        if (!b) return a;
        assert(!a->olderSibling);
        assert(!b->olderSibling);
        if (comp(b->d, a->d)) std::swap(a, b);
        if (a->youngestChild) {
            b->olderSibling = a->youngestChild;
            a->youngestChild = b;
        } else {
            a->youngestChild = b;
        }
        b->parentOrYoungerSibling = a;
        a->parentOrYoungerSibling = nullptr;
        return a;
    }
    static Node* unlink(Node* node) {
        assert(node);
        assert(node->parentOrYoungerSibling);
        if (node == node->parentOrYoungerSibling->youngestChild)
            node->parentOrYoungerSibling->youngestChild = node->olderSibling;
        else
            node->parentOrYoungerSibling->olderSibling = node->olderSibling;
        node->parentOrYoungerSibling = nullptr;
        node->olderSibling = nullptr;
        return node;
    }

public:
    typedef Node* HeapItem;
    PairingHeap() : comp{}, root{nullptr} {}
    Node* findMin() const { return root; }
    Node* insert(T const& d) {
        root = link(root, new Node{d, nullptr, nullptr, nullptr});
        return !comp(root->d, d) && !comp(d, root->d) ? root : root->youngestChild;
    }
    void modify(Node* node, T const& newD) {
        node->d = newD;
        if (node != root) {
            unlink(node);
            root = link(root, node);
        }
    }
    void deleteMin() {
        Node* oldRoot = root;
        std::vector<Node*> allChildren;
        for (Node* i = root->youngestChild; i; i = i->olderSibling) allChildren.push_back(i);
        for (Node* i : allChildren) {
            i->olderSibling = nullptr;
            i->parentOrYoungerSibling = nullptr;
        }
        for (size_t i = 0; i < (allChildren.size() & (~1ull)); i += 2)
            allChildren[i] = link(allChildren[i], allChildren[i | 1]);
        root = (allChildren.size() & 1) ? allChildren.back() : nullptr;
        for (size_t i = (allChildren.size() & (~1ull)); i; i -= 2) root = link(root, allChildren[i - 2]);
        delete oldRoot;
    }
};
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

    static bool isGeoCoordOnSegment(GeoCoord const& gc, StreetSegment const& ss) {
        return gc == ss.segment.start || gc == ss.segment.end;
    }

    void reconstructPath(std::vector<GeoCoord> const& path, std::vector<NavSegment>&) const {
        fprintf(stderr, "GeoGraphics[{Red, Thick, GeoPath[{ {%s,%s}", path.front().latitudeText.c_str(),
                path.front().longitudeText.c_str());
        for (auto i = std::next(path.begin()); i != path.end(); ++i)
            fprintf(stderr, ",{%s,%s}", i->latitudeText.c_str(), i->longitudeText.c_str());
        fprintf(stderr, "}]}]\n");
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
        GeoCoord startCoord;
        if (!attractionMapper.getGeoCoord(start, startCoord)) return NAV_BAD_SOURCE;
        fprintf(stderr, "Found attraction %s at {%s,%s}\n", start.c_str(), startCoord.latitudeText.c_str(),
                startCoord.longitudeText.c_str());
        GeoCoord endCoord;
        if (!attractionMapper.getGeoCoord(end, endCoord)) return NAV_BAD_DESTINATION;

        auto endStreetSegments = segmentMapper.getSegments(endCoord);
        assert(endStreetSegments.size() == 1);
        auto endStreetSegment = endStreetSegments.front();

        auto startStreetSegments = segmentMapper.getSegments(startCoord);
        assert(startStreetSegments.size() == 1);
        auto startStreetSegment = startStreetSegments.front();

        std::map<GeoCoord, GeoCoord> evaluatedNodes;
        std::map<GeoCoord, DiscoveredNode> discoveredNodes;

        {
            double travelledToStart = distanceEarthKM(startCoord, startStreetSegment.segment.start);
            discoveredNodes.emplace(
              startStreetSegment.segment.start,
              DiscoveredNode(startStreetSegment.segment.start, travelledToStart,
                             travelledToStart + distanceEarthKM(startStreetSegment.segment.start, endCoord)));
        }
        {
            double travelledToEnd = distanceEarthKM(startCoord, startStreetSegment.segment.end);
            discoveredNodes.emplace(
              startStreetSegment.segment.end,
              DiscoveredNode(startStreetSegment.segment.end, travelledToEnd,
                             travelledToEnd + distanceEarthKM(startStreetSegment.segment.end, endCoord)));
        }

        while (1) {
            auto currentIt =
              std::min_element(discoveredNodes.begin(), discoveredNodes.end(), [](auto const& a, auto const& b) {
                  return a.second.optimisticEstimate < b.second.optimisticEstimate;
              });
            if (currentIt->second.optimisticEstimate == HUGE_VAL) break;
            fprintf(stderr, "Relaxing edges starting from {%s,%s}\n", currentIt->first.latitudeText.c_str(),
                    currentIt->first.longitudeText.c_str());
            if (isGeoCoordOnSegment(currentIt->first, endStreetSegment)) {
                std::vector<GeoCoord> path;
                path.emplace_back(endCoord);
                GeoCoord here = currentIt->first;
                while (1) {
                    path.emplace_back(here);
                    if (isGeoCoordOnSegment(here, startStreetSegment)) break;
                    auto i = discoveredNodes.find(here);
                    auto j = evaluatedNodes.find(here);
                    if (i != discoveredNodes.end())
                        here = i->second.parent;
                    else if (j != evaluatedNodes.end())
                        here = j->second;
                    else
                        assert(false && "cannot find parent node");
                }
                path.emplace_back(startCoord);
                std::reverse(path.begin(), path.end());
                reconstructPath(path, directions);
                return NAV_SUCCESS;
            }

            currentIt->second.optimisticEstimate = HUGE_VAL;
            evaluatedNodes.emplace(currentIt->first, currentIt->second.parent);
            for (auto const& neighbor : getNeighbors(currentIt->first)) {
                if (evaluatedNodes.find(neighbor) != evaluatedNodes.end()) {
                    fprintf(stderr, "Skipping coord {%s,%s} because it has already been evaluated.\n",
                            neighbor.latitudeText.c_str(), neighbor.longitudeText.c_str());
                    continue;
                }
                fprintf(stderr, "Investigating GeoGraphics[{Red,Thick,GeoPath[{{%s,%s},{%s,%s}}]}]\n",
                        currentIt->first.latitudeText.c_str(), currentIt->first.longitudeText.c_str(),
                        neighbor.latitudeText.c_str(), neighbor.longitudeText.c_str());
                double distance = currentIt->second.travelledDistance + distanceEarthKM(currentIt->first, neighbor);
                assert(distance > 0);
                auto it = discoveredNodes.find(neighbor);
                if (it == discoveredNodes.end())
                    discoveredNodes.emplace(neighbor, DiscoveredNode(currentIt->first, distance,
                                                                     distance + distanceEarthKM(neighbor, endCoord)));
                else if (distance >= it->second.travelledDistance)
                    continue;
                else
                    it->second =
                      DiscoveredNode(currentIt->first, distance, distance + distanceEarthKM(neighbor, endCoord));
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
