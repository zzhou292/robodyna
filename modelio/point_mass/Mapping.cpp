#include "Mapping.h"
#include "output/ArtifactIO.h"

namespace crash::modelio::point_mass::detail {
MappedRecords Map(const std::vector<physical_scope::rigid::point_mass::Record>& records,
                  const tl::fea::NodalNodeDomain& domain, std::size_t cap) {
    output::Require(domain.prepared() && cap && cap <= Limits{}.native.max_records && records.size() <= cap,
                    "Point-mass mapping domain or source extent is invalid");
    MappedRecords result;
    result.dispositions.reserve(records.size());
    result.retained.reserve(records.size());
    for (std::size_t row = 0; row < records.size(); ++row) {
        const auto& value = records[row].value;
        const auto node = domain.Find(value.source_node_id);
        result.dispositions.push_back({row, node});
        if (node != SIZE_MAX)
            result.retained.push_back({value.source_element_id, value.source_node_id, node, value.supplied_mass_source});
    }
    return result;
}
} // namespace crash::modelio::point_mass::detail
