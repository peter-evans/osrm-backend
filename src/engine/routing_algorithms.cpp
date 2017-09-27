#include "engine/routing_algorithms.hpp"

namespace osrm
{
namespace engine
{

template <typename Algorithm>
std::vector<EdgeDuration>
RoutingAlgorithms<Algorithm>::ManyToManySearch(const std::vector<PhantomNode> &phantom_nodes,
                                               const std::vector<std::size_t> &source_indices,
                                               const std::vector<std::size_t> &target_indices) const
{
    return routing_algorithms::manyToManySearch(
               heaps, *facade, phantom_nodes, source_indices, target_indices)
        .second;
}

template std::vector<EdgeDuration>
RoutingAlgorithms<routing_algorithms::ch::Algorithm>::ManyToManySearch(
    const std::vector<PhantomNode> &phantom_nodes,
    const std::vector<std::size_t> &source_indices,
    const std::vector<std::size_t> &target_indices) const;

template std::vector<EdgeDuration>
RoutingAlgorithms<routing_algorithms::corech::Algorithm>::ManyToManySearch(
    const std::vector<PhantomNode> &phantom_nodes,
    const std::vector<std::size_t> &source_indices,
    const std::vector<std::size_t> &target_indices) const;

// One-to-many and many-to-one can be handled with MLD separately from many-to-many search.
// One-to-many (many-to-one) search is a unidirectional forward (backward) Dijkstra search
// with the candidate node level min(GetQueryLevel(phantom_node, phantom_nodes, node)
template <>
std::vector<EdgeDuration> RoutingAlgorithms<routing_algorithms::mld::Algorithm>::ManyToManySearch(
    const std::vector<PhantomNode> &phantom_nodes,
    const std::vector<std::size_t> &source_indices,
    const std::vector<std::size_t> &target_indices) const
{
    auto res = routing_algorithms::manyToManySearch(
        heaps, *facade, phantom_nodes, source_indices, target_indices);

    if (source_indices.size() == 1)
    { // TODO: check if target_indices.size() == 1 and do a bi-directional search
        auto r = routing_algorithms::mld::oneToManySearch<routing_algorithms::FORWARD_DIRECTION>(
            heaps, *facade, phantom_nodes, source_indices.front(), target_indices);

        for (std::size_t i = 0; i < r.first.size(); ++i)
        {
            if (res.first[i] != r.first[i] || res.second[i] != r.second[i])
            {
                std::cout << "m2m result\n";
                std::cout << "weights = ";
                for (std::size_t j = 0; j < res.first.size(); ++j)
                    std::cout << " " << res.first[j];
                std::cout << "\n";
                std::cout << "durations = ";
                for (std::size_t j = 0; j < res.second.size(); ++j)
                    std::cout << " " << res.second[j];
                std::cout << "\n";
                std::cout << "o2m result\n";
                std::cout << "weights = ";
                for (std::size_t j = 0; j < r.first.size(); ++j)
                    std::cout << " " << r.first[j];
                std::cout << "\n";
                std::cout << "durations = ";
                for (std::size_t j = 0; j < r.second.size(); ++j)
                    std::cout << " " << r.second[j];
                std::cout << "\n";
                exit(1);
            }
        }

        return r.second;
    }

    if (target_indices.size() == 1)
    {
        auto r = routing_algorithms::mld::oneToManySearch<routing_algorithms::REVERSE_DIRECTION>(
            heaps, *facade, phantom_nodes, target_indices.front(), source_indices);

        for (std::size_t i = 0; i < r.first.size(); ++i)
        {
            if (res.first[i] != r.first[i] || res.second[i] != r.second[i])
            {
                std::cout << "m2m result\n";
                std::cout << "weights = ";
                for (std::size_t j = 0; j < res.first.size(); ++j)
                    std::cout << " " << res.first[j];
                std::cout << "\n";
                std::cout << "durations = ";
                for (std::size_t j = 0; j < res.second.size(); ++j)
                    std::cout << " " << res.second[j];
                std::cout << "\n";
                std::cout << "o2m result\n";
                std::cout << "weights = ";
                for (std::size_t j = 0; j < r.first.size(); ++j)
                    std::cout << " " << r.first[j];
                std::cout << "\n";
                std::cout << "durations = ";
                for (std::size_t j = 0; j < r.second.size(); ++j)
                    std::cout << " " << r.second[j];
                std::cout << "\n";
                exit(1);
            }
        }

        return r.second;
    }

    return routing_algorithms::manyToManySearch(
               heaps, *facade, phantom_nodes, source_indices, target_indices)
        .second;
}

} // namespace engine
} // namespace osrm
