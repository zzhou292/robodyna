#include "Internal.h"
#include <iterator>
#include <type_traits>

namespace crash::modelio::tied_shell::search_detail {
namespace {
void Add(std::size_t& bytes, std::size_t count, std::size_t width, std::size_t cap) {
    Require(width && bytes <= cap && count <= (cap-bytes)/width,
            "Tied search geometry host byte cap exceeded");
    bytes += count*width;
}
}
std::size_t Preflight(const source::CanonicalData& s, const Data& d,
        const PackingData& p, SearchGeometryLimits limits) {
    const SearchGeometryLimits hard;
    const std::size_t values[] = {limits.host_bytes, limits.member_bytes, limits.metadata_bytes,
        limits.masters, limits.nodes, limits.shells, limits.parts, limits.matches_per_master};
    const std::size_t maximum[] = {hard.host_bytes, hard.member_bytes, hard.metadata_bytes,
        hard.masters, hard.nodes, hard.shells, hard.parts, hard.matches_per_master};
    for (unsigned i = 0; i < std::size(values); ++i)
        Require(values[i] && values[i] <= maximum[i], "Invalid tied search geometry limits");
    Require(s.inputs.source_member.bytes && s.inputs.source_member.bytes <= limits.member_bytes &&
            s.canonical_nodes <= limits.nodes && s.canonical_shells <= limits.shells &&
            s.parts.size() <= limits.parts && !d.masters.empty() && d.masters.size() <= limits.masters &&
            p.master_rows.size() == d.masters.size() && p.master_ranks.size() == d.masters.size() &&
            d.master_nodes.size() <= s.canonical_nodes && d.slave_nodes.size() <= s.canonical_nodes,
            "Tied search geometry source count exceeds capacity");
    std::size_t bytes = sizeof(SearchGeometryData) + sizeof(source::CanonicalData) + 256;
    Add(bytes, d.owned_payload_bytes, 1, limits.host_bytes);
    Add(bytes, p.owned_payload_bytes, 1, limits.host_bytes);
    for (const auto& a : s.arrays) Add(bytes, a.bytes.size(), 1, limits.host_bytes);
    Add(bytes, s.parts.capacity(), sizeof(source::PartDeclaration), limits.host_bytes);
    Add(bytes, s.selected_parts.capacity()+s.excluded_parts.capacity(), sizeof(SourceId), limits.host_bytes);
    // Both metadata DOMs, original bytes and the borrowed member coexist with
    // topology decoding. Canonical and declaration backing are charged once.
    Add(bytes, s.canonical_bytes.size(), 7, limits.host_bytes);
    Add(bytes, s.scope_bytes.size(), 7, limits.host_bytes);
    Add(bytes, s.inputs.source_member.bytes, 1, limits.host_bytes);
    Add(bytes, limits.metadata_bytes, 4, limits.host_bytes);
    // Complete decoded topology and decoder temporary; sorted shell keys,
    // per-node masks/remapping, final working nodes and selected card metadata.
    Add(bytes, s.canonical_shells, 144, limits.host_bytes);
    Add(bytes, s.canonical_nodes, 112, limits.host_bytes);
    Add(bytes, d.masters.size(), sizeof(SearchMasterGeometry), limits.host_bytes);
    Add(bytes, d.masters.size(), limits.matches_per_master*sizeof(SearchShellMatch), limits.host_bytes);
    Add(bytes, d.slave_nodes.size(), sizeof(std::uint32_t), limits.host_bytes);
    Add(bytes, limits.parts, sizeof(SearchProperty)+1024, limits.host_bytes);
    return bytes;
}
std::size_t OwnedPayload(const SearchGeometryData& d, SearchGeometryLimits limits) {
    std::size_t bytes = sizeof(d);
    const auto vector = [&](const auto& values) {
        Add(bytes, values.capacity(), sizeof(typename std::decay_t<decltype(values)>::value_type), limits.host_bytes);
    };
    const auto text = [&](const std::string& value) { Add(bytes, value.capacity()+1, 1, limits.host_bytes); };
    vector(d.canonical_nodes);
    vector(d.working_positions);
    vector(d.secondary_working_nodes);
    vector(d.masters);
    vector(d.matches);
    vector(d.properties);
    vector(d.sources);
    for (const auto& source : d.sources) {
        text(source.block.filename);
        text(source.block.keyword);
        text(source.block.raw_text);
        text(source.block.sha256);
        vector(source.cards);
        for (const auto& card : source.cards) text(card.second);
    }
    return bytes; // Payload, not allocator/control-block or RSS accounting.
}
} // namespace crash::modelio::tied_shell
