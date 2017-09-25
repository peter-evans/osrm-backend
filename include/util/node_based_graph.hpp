#ifndef NODE_BASED_GRAPH_HPP
#define NODE_BASED_GRAPH_HPP

#include "extractor/class_data.hpp"
#include "extractor/guidance/road_classification.hpp"
#include "extractor/node_based_edge.hpp"
#include "extractor/node_data_container.hpp"
#include "util/dynamic_graph.hpp"
#include "util/graph_utils.hpp"

#include <tbb/parallel_sort.h>

#include <memory>
#include <utility>

namespace osrm
{
namespace util
{

struct NodeBasedEdgeData
{
    NodeBasedEdgeData()
        : weight(INVALID_EDGE_WEIGHT), duration(INVALID_EDGE_WEIGHT), edge_id(SPECIAL_NODEID),
          reversed(false), shared_data_id(-1)
    {
    }

    NodeBasedEdgeData(EdgeWeight weight,
                      EdgeWeight duration,
                      unsigned edge_id,
                      bool reversed,
                      extractor::SharedDataID shared_data_id)
        : weight(weight), duration(duration), edge_id(edge_id), reversed(reversed),
          shared_data_id(shared_data_id)
    {
    }

    EdgeWeight weight;
    EdgeWeight duration;
    unsigned edge_id;
    bool reversed : 1;
    extractor::SharedDataID shared_data_id;

    bool IsCompatibleTo(const NodeBasedEdgeData &other) const
    {
        return (reversed == other.reversed) && shared_data_id == other.shared_data_id;
    }

    bool CanCombineWith(const NodeBasedEdgeData &other) const { return IsCompatibleTo(other); }
};

using NodeBasedDynamicGraph = DynamicGraph<NodeBasedEdgeData>;

/// Factory method to create NodeBasedDynamicGraph from NodeBasedEdges
/// Since DynamicGraph expects directed edges, we need to insert
/// two edges for undirected edges.
// inline std::pair<std::shared_ptr<NodeBasedDynamicGraph>,
//                 std::shared_ptr<extractor::EdgeBasedNodeDataContainer>>
inline std::shared_ptr<NodeBasedDynamicGraph>
NodeBasedDynamicGraphFromEdges(NodeID number_of_nodes,
                               const std::vector<extractor::NodeBasedEdge> &input_edge_list)
{
    auto edges_list = directedEdgesFromCompressed<NodeBasedDynamicGraph::InputEdge>(
        input_edge_list,
        [](NodeBasedDynamicGraph::InputEdge &output_edge,
           const extractor::NodeBasedEdge &input_edge) {
            output_edge.data.weight = input_edge.weight;
            output_edge.data.duration = input_edge.duration;
            output_edge.data.shared_data_id = input_edge.shared_data_id;

            BOOST_ASSERT(output_edge.data.weight > 0);
            BOOST_ASSERT(output_edge.data.duration > 0);
        });

    tbb::parallel_sort(edges_list.begin(), edges_list.end());

    auto graph = std::make_shared<NodeBasedDynamicGraph>(number_of_nodes, edges_list);
    return graph;
    //   auto graph_meta_data = std::make_shared<extractor::EdgeBasedNodeDataContainer>(edges_list);

    //   return std::make_pair(graph, graph_meta_data);
}
}
}

#endif // NODE_BASED_GRAPH_HPP
