#include "Internal.h"
#include "output/ArtifactIO.h"
#include <algorithm>
#include <climits>
#include <numeric>
#include <stdexcept>

namespace crash::modelio::tied_shell::packing_detail {
namespace {
template<class T> std::vector<T> Decode(const source::CanonicalData& source, const char* name) {
    const auto& array = source::FindArray(source, name);
    return output::arrays::Decode<T>(array.descriptor, array.bytes);
}
void CheckNodes(const Data& d, const std::vector<SourceId>& ids) {
    for (const auto id : ids)
        output::Require(id && id <= INT_MAX, "Tied packing requires positive native-range source NIDs");
    SourceId previous = 0;
    for (const auto& node : d.slave_nodes) {
        output::Require(node.canonical_index < ids.size() &&
                        ids[node.canonical_index] == node.id && node.id > previous,
                        "Tied packing slave node identity/order changed");
        previous = node.id;
    }
    previous = 0;
    for (const auto index : d.master_nodes) {
        output::Require(index < ids.size() && ids[index] > previous,
                        "Tied packing master node identity/order changed");
        previous = ids[index];
    }
}
}
PackingData Build(const source::CanonicalData& source, const Data& d, PackingLimits limits) {
    PackingData next;
    next.startup_budget_bytes = Preflight(source, d, limits);
    CheckPolicy(d);
    const auto records = Decode<SourceId>(source, "shells_records");
    const auto ids = Decode<SourceId>(source, "node_ids");
    output::Require(records.size() == 6*source.canonical_shells &&
                    ids.size() == source.canonical_nodes,
                    "Tied packing decoded source extent changed");
    CheckNodes(d, ids);
    for (const auto& parent : d.masters) {
        output::Require(parent.canonical_index < source.canonical_shells &&
                        parent.part_index < d.parts.size() && parent.family == ElementFamily::Shell,
                        "Tied packing master source row changed");
        const auto* row = records.data()+6*parent.canonical_index;
        output::Require(row[0] == parent.id && row[1] == d.parts[parent.part_index].id &&
                        row[2] && row[3] && row[4] && row[5] &&
                        row[2] != row[3] && row[2] != row[4] && row[2] != row[5] &&
                        row[3] != row[4] && row[3] != row[5] &&
                        parent.arity == (row[4] == row[5] ? 3u : 4u),
                        "Tied packing requires exact true Q4/T3 source topology");
    }
    const auto tuple = [&](std::uint32_t index) {
        return records.data()+6*d.masters[index].canonical_index+2;
    };
    next.master_rows.resize(d.masters.size());
    std::iota(next.master_rows.begin(), next.master_rows.end(), 0u);
    std::sort(next.master_rows.begin(), next.master_rows.end(), [&](auto a, auto b) {
        return std::lexicographical_compare(tuple(a), tuple(a)+4, tuple(b), tuple(b)+4);
    });
    for (std::size_t rank = 1; rank < next.master_rows.size(); ++rank) {
        const auto a = next.master_rows[rank-1];
        const auto b = next.master_rows[rank];
        if (std::equal(tuple(a), tuple(a)+4, tuple(b))) {
            throw std::runtime_error("Tied packing ambiguous tuple needs native registration: EIDs " +
                std::to_string(d.masters[a].id) + ", " + std::to_string(d.masters[b].id));
        }
    }
    next.master_ranks.resize(d.masters.size());
    for (std::size_t rank = 0; rank < next.master_rows.size(); ++rank)
        next.master_ranks[next.master_rows[rank]] = static_cast<std::uint32_t>(rank);
    next.owned_payload_bytes = sizeof(PackingData) +
        (next.master_rows.capacity()+next.master_ranks.capacity())*sizeof(std::uint32_t);
    return next;
}
} // namespace crash::modelio::tied_shell::packing_detail
