#ifndef NODE_BASED_EDGE_HPP
#define NODE_BASED_EDGE_HPP

#include <cstdint>
#include <tuple>

#include "extractor/class_data.hpp"
#include "extractor/travel_mode.hpp"
#include "util/typedefs.hpp"

#include "extractor/guidance/road_classification.hpp"

namespace osrm
{
namespace extractor
{

using SharedDataID = std::uint32_t;

// Shared Data which is the same for many NodeBasedEdges
struct NodeBasedEdgeSharedData
{
    NameID name_id;                                   // 32 4
    std::uint8_t roundabout : 1;                      // 1
    std::uint8_t circular : 1;                        // 1
    std::uint8_t startpoint : 1;                      // 1
    std::uint8_t restricted : 1;                      // 1
    TravelMode travel_mode : 4;                       // 4
    ClassData classes;                                // 8  1
    LaneDescriptionID lane_description_id;            // 16 2
    guidance::RoadClassification road_classification; // 16 2
    GeometryID geometry_id;                           // 32 4

    bool CanCombineWith(const NodeBasedEdgeSharedData &other) const
    {
        return (std::tie(name_id, road_classification) ==
                std::tie(other.name_id, other.road_classification)) &&
               (roundabout == other.roundabout) && (circular == other.circular) &&
               (startpoint == other.startpoint) && (restricted == other.restricted) &&
               (travel_mode == other.travel_mode);
    }
};

struct NodeBasedEdge
{
    NodeBasedEdge();

    NodeBasedEdge(NodeID source,
                  NodeID target,
                  EdgeWeight weight,
                  EdgeDuration duration,
                  bool forward,
                  bool backward,
                  bool is_split,
                  SharedDataID shared_data_id);

    bool operator<(const NodeBasedEdge &other) const;

    NodeID source;               // 32 4
    NodeID target;               // 32 4
    EdgeWeight weight;           // 32 4
    EdgeDuration duration;       // 32 4
    std::uint8_t forward : 1;    // 1
    std::uint8_t backward : 1;   // 1
    std::uint8_t is_split : 1;   // 1
    SharedDataID shared_data_id; // 32 4
};

struct NodeBasedEdgeWithOSM : NodeBasedEdge
{
    NodeBasedEdgeWithOSM(OSMNodeID source,
                         OSMNodeID target,
                         EdgeWeight weight,
                         EdgeDuration duration,
                         bool forward,
                         bool backward,
                         bool is_split,
                         SharedDataID shared_data_id);

    OSMNodeID osm_source_id;
    OSMNodeID osm_target_id;
};

// Impl.

inline NodeBasedEdge::NodeBasedEdge()
    : source(SPECIAL_NODEID), target(SPECIAL_NODEID), weight(0), duration(0), forward(false),
      backward(false), is_split(false), shared_data_id(-1)
{
}

inline NodeBasedEdge::NodeBasedEdge(NodeID source,
                                    NodeID target,
                                    EdgeWeight weight,
                                    EdgeDuration duration,
                                    bool forward,
                                    bool backward,
                                    bool is_split,
                                    SharedDataID shared_data_id)
    : source(source), target(target), weight(weight), duration(duration), forward(forward),
      backward(backward), is_split(is_split), shared_data_id(shared_data_id)
{
}

inline bool NodeBasedEdge::operator<(const NodeBasedEdge &other) const
{
    if (source == other.source)
    {
        if (target == other.target)
        {
            if (weight == other.weight)
            {
                return forward && backward && ((!other.forward) || (!other.backward));
            }
            return weight < other.weight;
        }
        return target < other.target;
    }
    return source < other.source;
}

inline NodeBasedEdgeWithOSM::NodeBasedEdgeWithOSM(OSMNodeID source,
                                                  OSMNodeID target,
                                                  EdgeWeight weight,
                                                  EdgeDuration duration,
                                                  bool forward,
                                                  bool backward,
                                                  bool is_split,
                                                  SharedDataID shared_data_id)
    : NodeBasedEdge(SPECIAL_NODEID,
                    SPECIAL_NODEID,
                    weight,
                    duration,
                    forward,
                    backward,
                    is_split,
                    shared_data_id),
      osm_source_id(std::move(source)), osm_target_id(std::move(target))
{
}

static_assert(sizeof(extractor::NodeBasedEdge) == 24,
              "Size of extractor::NodeBasedEdge type is "
              "bigger than expected. This will influence "
              "memory consumption.");

} // ns extractor
} // ns osrm

#endif /* NODE_BASED_EDGE_HPP */
