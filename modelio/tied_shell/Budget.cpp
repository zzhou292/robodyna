#include "Internal.h"
#include <algorithm>
#include <iterator>
#include <type_traits>

namespace crash::modelio::tied_shell::detail {
namespace {
void Add(std::size_t& total, std::size_t count, std::size_t width, std::size_t cap) {
    Require(width && total <= cap && count <= (cap-total)/width,
            "Tied declaration host byte cap exceeded");
    total += count*width;
}
}
std::size_t Preflight(const source::CanonicalData& d, Limits limits) {
    const Limits maximum;
    const std::size_t supplied[] = {limits.host_bytes, limits.member_bytes, limits.metadata_bytes,
        limits.masters, limits.slave_elements, limits.slave_nodes, limits.parts,
        limits.groups, limits.group_members, limits.blocks};
    const std::size_t allowed[] = {maximum.host_bytes, maximum.member_bytes, maximum.metadata_bytes,
        maximum.masters, maximum.slave_elements, maximum.slave_nodes, maximum.parts,
        maximum.groups, maximum.group_members, maximum.blocks};
    for (unsigned i = 0; i < std::size(supplied); ++i)
        Require(supplied[i] && supplied[i] <= allowed[i], "Invalid tied declaration limits");
    Require(d.inputs.source_member.bytes && d.inputs.source_member.bytes <= limits.member_bytes &&
            d.canonical_nodes <= UINT32_MAX && d.canonical_shells <= UINT32_MAX,
            "Tied declaration source extent exceeds capacity");
    std::size_t total = sizeof(Data) + sizeof(source::CanonicalData) + 256;
    // Shared immutable canonical payload is charged once. Input member bytes,
    // parser DOMs, decode scratch and staged output coexist during preparation.
    for (const auto& array : d.arrays) Add(total, array.bytes.size(), 1, limits.host_bytes);
    Add(total, d.parts.capacity(), sizeof(source::PartDeclaration), limits.host_bytes);
    Add(total, d.selected_parts.capacity(), sizeof(SourceId), limits.host_bytes);
    Add(total, d.excluded_parts.capacity(), sizeof(SourceId), limits.host_bytes);
    Add(total, d.canonical_bytes.size(), 7, limits.host_bytes);
    Add(total, d.scope_bytes.size(), 7, limits.host_bytes);
    Add(total, d.inputs.source_member.bytes, 1, limits.host_bytes);
    Add(total, limits.metadata_bytes, 4, limits.host_bytes);
    Add(total, d.canonical_nodes, 40, limits.host_bytes);
    Add(total, d.canonical_shells, 112, limits.host_bytes);
    Add(total, limits.slave_elements, sizeof(Element)+8*sizeof(Incidence), limits.host_bytes);
    Add(total, limits.slave_nodes, sizeof(Node), limits.host_bytes);
    Add(total, limits.parts, sizeof(Part)+128, limits.host_bytes);
    Add(total, limits.groups, sizeof(GroupEvidence), limits.host_bytes);
    Add(total, limits.group_members, 3*sizeof(SourceId)+64, limits.host_bytes);
    Add(total, limits.blocks, sizeof(Request)+sizeof(UnresolvedBlock)+128, limits.host_bytes);
    return total;
}
std::size_t OwnedPayload(const Data& d, std::size_t cap) {
    std::size_t total = sizeof(Data);
    const auto text = [&](const std::string& s) { Add(total, s.capacity()+1, 1, cap); };
    const auto vector = [&](const auto& v) {
        Add(total, v.capacity(), sizeof(typename std::decay_t<decltype(v)>::value_type), cap);
    };
    vector(d.slave_part_ids);
    vector(d.master_part_ids);
    vector(d.sources);
    vector(d.parts);
    vector(d.masters);
    vector(d.master_nodes);
    vector(d.slaves);
    vector(d.incidences);
    vector(d.slave_nodes);
    vector(d.groups);
    vector(d.unresolved_constraints);
    for (const auto& source : d.sources) {
        text(source.block.filename);
        text(source.block.keyword);
        text(source.block.sha256);
        text(source.block.raw_text);
        vector(source.cards);
        for (const auto& card : source.cards) text(card.second);
    }
    for (const auto& g : d.groups) {
        vector(g.source_nodes);
        vector(g.slave_nodes);
        vector(g.master_nodes);
    }
    for (const auto& block : d.unresolved_constraints) {
        text(block.filename);
        text(block.keyword);
        text(block.sha256);
    }
    return total; // Own payload only; excludes shared canonical, allocator/control block and RSS.
}
} // namespace crash::modelio::tied_shell::detail
