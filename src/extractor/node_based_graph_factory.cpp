#include "extractor/node_based_graph_factory.hpp"
#include "extractor/graph_compressor.hpp"
#include "storage/io.hpp"
#include "util/graph_loader.hpp"

#include "util/log.hpp"

namespace osrm
{
namespace extractor
{

NodeBasedGraphFactory::NodeBasedGraphFactory(
    const boost::filesystem::path &input_file,
    ScriptingEnvironment &scripting_environment,
    std::vector<TurnRestriction> &turn_restrictions,
    std::vector<ConditionalTurnRestriction> &conditional_turn_restrictions)
{
    LoadDataFromFile(input_file);
    Compress(scripting_environment, turn_restrictions, conditional_turn_restrictions);
}

// load the data serialised during the extraction run
void NodeBasedGraphFactory::LoadDataFromFile(const boost::filesystem::path &input_file)
{
    // the extraction_containers serialise all data necessary to create the node-based graph into a
    // single file, the *.osrm file. It contains nodes, basic information about which of these nodes
    // are traffic signals/stop signs. It also contains Edges and purely annotative meta-data
    storage::io::FileReader file_reader(input_file, storage::io::FileReader::VerifyFingerprint);

    auto barriers_iter = inserter(barriers, end(barriers));
    auto traffic_signals_iter = inserter(traffic_signals, end(traffic_signals));

    const auto number_of_node_based_nodes = util::loadNodesFromFile(
        file_reader, barriers_iter, traffic_signals_iter, coordinates, osm_node_ids);

    std::vector<NodeBasedEdge> edge_list;
    util::loadEdgesFromFile(file_reader, edge_list);

    if (edge_list.empty())
    {
        throw util::exception("Node-based-graph (" + input_file.string() + ") contains no edges." +
                              SOURCE_REF);
    }

    util::loadAnnotationData(file_reader, annotation_data);

    // at this point, the data isn't compressed, but since we update the graph in-place, we assign
    // it here.
    compressed_output_graph =
        util::NodeBasedDynamicGraphFromEdges(number_of_node_based_nodes, edge_list);
}

void NodeBasedGraphFactory::Compress(
    ScriptingEnvironment &scripting_environment,
    std::vector<TurnRestriction> &turn_restrictions,
    std::vector<ConditionalTurnRestriction> &conditional_turn_restrictions)
{
    GraphCompressor graph_compressor;
    graph_compressor.Compress(barriers,
                              traffic_signals,
                              scripting_environment,
                              turn_restrictions,
                              conditional_turn_restrictions,
                              *compressed_output_graph,
                              annotation_data,
                              compressed_edge_container);

    // TODO remap node-based graph shared data index / remove unused entries (from compression)

    // assign geometries for all edges
    // const unsigned packed_geometry_id = m_compressed_edge_container.ZipEdges(edge_id_1,
    // edge_id_2);
}

void NodeBasedGraphFactory::CompressGeometry()
{
    for (const auto nbg_node_u : util::irange(0u, compressed_output_graph->GetNumberOfNodes()))
    {
        BOOST_ASSERT(nbg_node_u != SPECIAL_NODEID);
        for (EdgeID nbg_edge_id : compressed_output_graph->GetAdjacentEdgeRange(nbg_node_u))
        {
            BOOST_ASSERT(nbg_edge_id != SPECIAL_EDGEID);

            const auto &nbg_edge_data = compressed_output_graph->GetEdgeData(nbg_edge_id);
            const auto nbg_node_v = compressed_output_graph->GetTarget(nbg_edge_id);
            BOOST_ASSERT(nbg_node_v != SPECIAL_NODEID);
            BOOST_ASSERT(nbg_node_u != nbg_node_v);

            // pick only every other edge, since we have every edge as an outgoing
            // and incoming egde
            if (nbg_node_u >= nbg_node_v)
            {
                continue;
            }

            auto from = nbg_node_u, to = nbg_node_v;
            // if we found a non-forward edge reverse and try again
            if (nbg_edge_data.edge_id == SPECIAL_NODEID)
                std::swap(from, to);

            // find forward edge id and
            const EdgeID edge_id_1 = compressed_output_graph->FindEdge(from, to);
            BOOST_ASSERT(edge_id_1 != SPECIAL_EDGEID);

            const auto &forward_data = compressed_output_graph->GetEdgeData(edge_id_1);

            // find reverse edge id and
            const EdgeID edge_id_2 = compressed_output_graph->FindEdge(to, from);
            BOOST_ASSERT(edge_id_2 != SPECIAL_EDGEID);

            auto packed_geometry_id = compressed_edge_container.ZipEdges(edge_id_1, edge_id_2);
        }
    }
}

} // namespace extractor
} // namespace osrm
