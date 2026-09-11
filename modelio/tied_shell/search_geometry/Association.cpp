#include "Internal.h"
#include <algorithm>
#include <limits>

namespace crash::modelio::tied_shell::search_detail {
Topology ReadTopology(const source::CanonicalData& source, SearchGeometryLimits limits) {
    Topology t;
    t.nodes = Decode<SourceId>(source, "node_ids");
    t.shells = Decode<SourceId>(source, "shells_records");
    t.connectivity = Decode<std::uint32_t>(source, "shells_node_indices");
    Require(t.nodes.size() == source.canonical_nodes && t.shells.size() == 6*source.canonical_shells &&
            t.connectivity.size() == 4*source.canonical_shells && t.nodes.size() <= limits.nodes,
            "Tied search complete canonical topology extent changed");
    for (std::size_t row = 0; row < source.canonical_shells; ++row) {
        for (unsigned local = 0; local < 4; ++local) {
            const auto node = t.connectivity[4*row+local];
            Require(node < t.nodes.size() && t.nodes[node] == t.shells[6*row+2+local],
                    "Tied search canonical connectivity/source identity changed");
        }
    }
    return t;
}
void Associate(const Topology& t, const Data& d, const PackingData& packing,
        SearchGeometryData& out, SearchGeometryLimits limits) {
    const auto nn = t.nodes.size(), ns = t.shells.size()/6;
    std::vector<std::uint32_t> offsets(nn+1, 0), remap(nn, UINT32_MAX);
    std::vector<unsigned char> roles(nn, 0);
    for (auto node : d.master_nodes) {
        Require(node < nn, "Tied master canonical node exceeds extent");
        roles[node] |= 1;
    }
    for (const auto& node : d.slave_nodes) {
        Require(node.canonical_index < nn && t.nodes[node.canonical_index] == node.id,
                "Tied secondary source association changed");
        roles[node.canonical_index] |= 2;
    }
    for (std::size_t row = 0; row < ns; ++row) {
        const unsigned arity = t.shells[6*row+4] == t.shells[6*row+5] ? 3 : 4;
        for (unsigned local = 0; local < arity; ++local) {
            const auto node = t.connectivity[4*row+local];
            Require(!(roles[node]&2), "Tied weld-only search secondary has original shell incidence");
            ++offsets[node+1];
        }
    }
    for (std::size_t i = 0; i < nn; ++i) offsets[i+1] += offsets[i];
    auto cursor = offsets;
    std::vector<std::uint32_t> incident(offsets.back());
    for (std::size_t row = 0; row < ns; ++row) {
        const unsigned arity = t.shells[6*row+4] == t.shells[6*row+5] ? 3 : 4;
        for (unsigned local = 0; local < arity; ++local)
            incident[cursor[t.connectivity[4*row+local]]++] = row;
    }
    out.canonical_nodes.reserve(d.master_nodes.size()+d.slave_nodes.size());
    for (std::size_t node = 0; node < nn; ++node) {
        if (!roles[node]) continue;
        remap[node] = out.canonical_nodes.size();
        out.canonical_nodes.push_back(node);
    }
    out.secondary_working_nodes.reserve(d.slave_nodes.size());
    for (const auto& node : d.slave_nodes)
        out.secondary_working_nodes.push_back(remap[node.canonical_index]);
    out.masters.reserve(d.masters.size());
    out.matches.reserve(d.masters.size());
    for (std::size_t rank = 0; rank < packing.master_rows.size(); ++rank) {
        const auto declaration_row = packing.master_rows[rank];
        Require(declaration_row < d.masters.size() && packing.master_ranks[declaration_row] == rank,
                "Tied search packing permutation changed");
        const auto& original = d.masters[declaration_row];
        Require(original.canonical_index < ns && t.shells[6*original.canonical_index] == original.id,
                "Tied search master source row changed");
        const auto* nodes = t.connectivity.data()+4*original.canonical_index;
        SearchMasterGeometry master;
        master.declaration_row = declaration_row;
        master.first_match = out.matches.size();
        master.family = original.arity == 3 ? SearchShellFamily::T3 : SearchShellFamily::Q4;
        for (unsigned local = 0; local < 4; ++local) master.working_nodes[local] = remap[nodes[local]];
        for (auto i = offsets[nodes[0]]; i < offsets[nodes[0]+1]; ++i) {
            const auto candidate = incident[i];
            const auto* other = t.connectivity.data()+4*candidate;
            const unsigned arity = other[2] == other[3] ? 3 : 4;
            if (arity == 3 && original.arity != 3) continue;
            bool matches = true;
            for (unsigned local = 0; local < original.arity; ++local)
                matches &= std::find(other, other+arity, nodes[local]) != other+arity;
            if (!matches) continue;
            Require(master.match_count < limits.matches_per_master && out.matches.size() < UINT32_MAX,
                    "Tied native matching shell count exceeds capacity");
            out.matches.push_back({candidate, 0, arity == 3 ? SearchShellFamily::T3 : SearchShellFamily::Q4, false});
            ++master.match_count;
        }
        Require(master.match_count, "Tied master has no original native-compatible shell");
        out.multiple_match_masters += master.match_count > 1;
        out.masters.push_back(master);
    }
}
} // namespace crash::modelio::tied_shell::search_detail
