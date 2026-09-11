#include "Internal.h"
#include <algorithm>

namespace crash::modelio::tied_shell::detail {
namespace {
template<class T> std::vector<T> Decode(const source::CanonicalData& source, const char* name) {
    const auto& a = source::FindArray(source, name);
    return output::arrays::Decode<T>(a.descriptor, a.bytes);
}
struct Family {
    const char* records;
    const char* connectivity;
    ElementFamily kind;
    unsigned width, slots;
};
}
void Geometry(const source::CanonicalData& source, const Value& scope, Draft& draft, Limits limits) {
    const auto ids = Decode<SourceId>(source, "node_ids");
    const auto codes = Decode<std::int32_t>(source, "node_codes");
    Require(ids.size() == source.canonical_nodes && codes.size() == 2*ids.size(),
            "Tied canonical node extent changed");
    std::vector<unsigned char> roles(ids.size(), 0);
    auto& d = draft.data;
    const Family families[] = {{"shells_records", "shells_node_indices", ElementFamily::Shell, 6, 4},
        {"beams_records", "beams_node_indices", ElementFamily::Beam, 10, 2},
        {"solids_records", "solids_node_indices", ElementFamily::Solid, 10, 8}};
    for (const auto& family : families) {
        const auto records = Decode<SourceId>(source, family.records);
        const auto connectivity = Decode<std::uint32_t>(source, family.connectivity);
        Require(records.size()%family.width == 0 &&
                connectivity.size() == (records.size()/family.width)*family.slots,
                "Tied canonical family extent changed");
        for (std::size_t i = 0; i < records.size()/family.width; ++i) {
            const auto* record = records.data()+family.width*i;
            const auto found = draft.part_rows.find(record[1]);
            if (found == draft.part_rows.end()) continue;
            const auto part_index = found->second;
            auto& p = d.parts[part_index];
            auto& count = family.kind == ElementFamily::Shell ? p.shells :
                          family.kind == ElementFamily::Beam ? p.beams : p.solids;
            ++count;
            const unsigned arity = family.kind == ElementFamily::Shell && record[4] == record[5] ? 3 : family.slots;
            const Element element{record[0], static_cast<std::uint32_t>(i),
                static_cast<std::uint32_t>(part_index), family.kind, arity};
            if (p.master) {
                Require(family.kind == ElementFamily::Shell, "Tied master part contains nonshell elements");
                Require(d.masters.size() < limits.masters, "Tied master parent count exceeds capacity");
                d.masters.push_back(element);
                ++(arity == 3 ? d.counts.t3 : d.counts.q4);
            }
            const auto slave_index = d.slaves.size();
            if (p.slave) {
                Require(slave_index < limits.slave_elements, "Tied slave element count exceeds capacity");
                d.slaves.push_back(element);
                auto& count = family.kind == ElementFamily::Shell ? d.counts.slave_shells :
                              family.kind == ElementFamily::Beam ? d.counts.slave_beams : d.counts.slave_solids;
                ++count;
            }
            // Retain all original connectivity slots, including repeated shell/solid slots.
            // Beam orientation N3 is outside its two physical endpoint slots.
            for (unsigned local = 0; local < family.slots; ++local) {
                const auto node = connectivity[family.slots*i+local];
                Require(node < ids.size() && ids[node] == record[2+local],
                        "Tied canonical connectivity differs from source identity");
                if (p.master) roles[node] |= 1;
                if (p.slave) {
                    roles[node] |= 2;
                    d.incidences.push_back({node, static_cast<std::uint32_t>(slave_index), local});
                }
            }
        }
    }
    for (std::size_t i = 0; i < roles.size(); ++i) {
        if (roles[i]&1) d.master_nodes.push_back(static_cast<std::uint32_t>(i));
        if (roles[i]&2) {
            Require(d.slave_nodes.size() < limits.slave_nodes, "Tied slave node count exceeds capacity");
            d.slave_nodes.push_back({ids[i], static_cast<std::uint32_t>(i), bool(roles[i]&1),
                {codes[2*i], codes[2*i+1]}});
            if (roles[i]&1) ++d.counts.shared_nodes;
        }
    }
    std::sort(d.master_nodes.begin(), d.master_nodes.end(), [&](auto a, auto b) { return ids[a] < ids[b]; });
    std::sort(d.slave_nodes.begin(), d.slave_nodes.end(), [](const Node& a, const Node& b) { return a.id < b.id; });
    for (auto node : d.master_nodes) draft.master_ids.push_back(ids[node]);
    for (const auto& node : d.slave_nodes) draft.slave_ids.push_back(node.id);
    const auto& expected = Member(Member(scope, "declarations"), "parts");
    std::size_t checked = 0;
    for (const auto& part : expected.GetArray()) {
        const auto found = draft.part_rows.find(Unsigned(part, "source_part_id"));
        if (found == draft.part_rows.end()) continue;
        ++checked;
        const auto& p = d.parts[found->second];
        const auto& counts = Member(part, "counts");
        Require(p.shells == Unsigned(counts, "shells") && p.beams == Unsigned(counts, "beams") &&
                p.solids == Unsigned(counts, "solids") && p.shells+p.beams+p.solids,
                "Late tied source part coverage mismatch");
    }
    Require(checked == d.parts.size() && !d.masters.empty() && !d.slaves.empty(), "Incomplete tied source coverage");
    d.counts.masters = d.masters.size();
    d.counts.master_nodes = d.master_nodes.size();
    d.counts.slave_nodes = d.slave_nodes.size();
    d.counts.incidences = d.incidences.size();
}
} // namespace crash::modelio::tied_shell::detail
