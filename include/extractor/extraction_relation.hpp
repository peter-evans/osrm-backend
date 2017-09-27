#ifndef EXTRACTION_RELATION_HPP
#define EXTRACTION_RELATION_HPP

#include <osmium/osm/relation.hpp>

#include <boost/assert.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace osrm
{
namespace extractor
{

struct ExtractionRelation
{
    using AttributesList = std::vector<std::pair<std::string, std::string>>;
    using OsmIDTyped = std::pair<osmium::object_id_type, osmium::item_type>;
    using MembersRolesList = std::vector<std::pair<std::uint64_t, std::string>>;

    struct OsmIDTypedHash
    {
        std::uint64_t operator()(const OsmIDTyped &id) const
        {
            return id.first ^ (static_cast<std::uint64_t>(id.second) << 56);
        }
    };

    void Clear()
    {
        attributes.clear();
        members_role.clear();
    }

    const char * GetAttr(const std::string & attr) const
    {
        auto it = std::lower_bound(
                    attributes.begin(),
                    attributes.end(),
                    std::make_pair(attr, std::string()));

        if (it != attributes.end() && (*it).first == attr)
            return (*it).second.c_str();

        return nullptr;
    }

    void Prepare()
    {
        std::sort(attributes.begin(), attributes.end());
        std::sort(members_role.begin(), members_role.end());
    }

    void AddMember(const OsmIDTyped & member_id, const char * role)
    {
        std::uint64_t hash = OsmIDTypedHash()(member_id);
        members_role.emplace_back(std::make_pair(hash, std::string(role)));
    }

    const char * GetRole(const OsmIDTyped & member_id) const
    {
        std::uint64_t hash = OsmIDTypedHash()(member_id);
        auto it = std::lower_bound(
                    members_role.begin(),
                    members_role.end(),
                    std::make_pair(hash, std::string()));

        if (it != members_role.end() && (*it).first == hash)
            return (*it).second.c_str();

        return nullptr;
    }

    osmium::object_id_type id;
    AttributesList attributes;
    MembersRolesList members_role;
};

// It contains data of all parsed relations for each node/way element
class ExtractionRelationContainer
{
  public:
    using AttributesMap = ExtractionRelation::AttributesList;
    using OsmIDTyped = ExtractionRelation::OsmIDTyped;
    using RelationList = std::vector<AttributesMap>;
    using RelationIDList = std::vector<std::uint64_t>;
    using RelationRefMap = std::unordered_map<std::uint64_t, RelationIDList>;

    void AddRelation(std::uint64_t rel_id, ExtractionRelation && rel)
    {
        rel.Prepare();

        BOOST_ASSERT(relations_data.find(rel_id) == relations_data.end());
        relations_data.insert(std::make_pair(rel_id, std::move(rel)));
    }

    void AddRelationMember(std::uint64_t rel_id, const OsmIDTyped & member_id)
    {
        switch (member_id.second)
        {
        case osmium::item_type::node:
            node_refs[rel_id].push_back(member_id.first);
            break;

        case osmium::item_type::way:
            way_refs[rel_id].push_back(member_id.first);
            break;

        case osmium::item_type::relation:
            rel_refs[rel_id].push_back(member_id.first);
            break;

        default:
            break;
        };
    }

    void Merge(ExtractionRelationContainer && other)
    {
        for (auto it : other.relations_data)
        {
            const auto res = relations_data.insert(std::make_pair(it.first, std::move(it.second)));
            BOOST_ASSERT(res.second);
            (void)res; // prevent unused warning in release
        }

        auto MergeRefMap = [&](RelationRefMap & source, RelationRefMap & target)
        {
            for (auto it : source)
            {
                auto & v = target[it.first];
                v.insert(v.end(), it.second.begin(), it.second.end());
            }
        };

        MergeRefMap(other.way_refs, way_refs);
        MergeRefMap(other.node_refs, node_refs);
        MergeRefMap(other.rel_refs, rel_refs);
    }

    std::size_t GetRelationsNum() const
    {
        return relations_data.size();
    }

    const RelationIDList & GetRelations(const OsmIDTyped & member_id) const
    {
        auto getFromMap = [this](osmium::object_id_type id, const RelationRefMap & map) -> const RelationIDList &
        {
            auto it = map.find(id);
            if (it != map.end())
                return it->second;

            return empty_rel_list;
        };

        switch (member_id.second)
        {
        case osmium::item_type::node:
            return getFromMap(member_id.first, node_refs);

        case osmium::item_type::way:
            return getFromMap(member_id.first, way_refs);

        case osmium::item_type::relation:
            return getFromMap(member_id.first, rel_refs);

        default:
            break;
        }

        return empty_rel_list;
    }

  private:
    RelationIDList empty_rel_list;
    std::unordered_map<std::uint64_t, ExtractionRelation> relations_data;

    // each map contains list of relation id's, that has keyed id as a member
    RelationRefMap way_refs;
    RelationRefMap node_refs;
    RelationRefMap rel_refs;
};

} // namespace extractor
} // namespace osrm

#endif // EXTRACTION_RELATION_HPP
