#include "extractor/node_based_graph_factory.hpp"
#include "extractor/graph_compressor.hpp"
#include "storage/io.hpp"
#include "util/graph_loader.hpp"

#include "util/log.hpp"

namespace osrm
{
namespace extractor
{

NodeBasedGraphFactory::NodeBasedGraphFactory(const boost::filesystem::path &input_file,
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

    util::Log() << " - " << barriers.size() << " bollard nodes, " << traffic_signals.size()
                << " traffic lights";

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

void NodeBasedGraphFactory::Compress(ScriptingEnvironment &scripting_environment,
                            std::vector<TurnRestriction> &turn_restrictions,
                          std::vector<ConditionalTurnRestriction> &conditional_turn_restrictions)
{
    GraphCompressor graph_compressor;
    graph_compressor.Compress(barriers,
                              traffic_signals,
                              scripting_environment,
                              turn_restrictions, conditional_turn_restrictions,
                              *compressed_output_graph,
                              annotation_data,
                              compressed_edge_container);


    // assign geometries for all edges
    // const unsigned packed_geometry_id = m_compressed_edge_container.ZipEdges(edge_id_1, edge_id_2);
}

} // namespace extractor
} // namespace osrm
