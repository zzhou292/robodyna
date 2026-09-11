#include "Internal.h"
#include <type_traits>

namespace crash::modelio::tied_shell::auxiliary_detail {
void Add(std::size_t& total, std::size_t count, std::size_t width, std::size_t cap) {
    Require(width && total <= cap && count <= (cap-total)/width, "Auxiliary host byte cap exceeded");
    total += count*width;
}
std::size_t Preflight(const source::CanonicalData& source, const Data& declaration,
                     std::size_t member_bytes, AuxiliaryLimits limits) {
    const AuxiliaryLimits maximum;
    const std::size_t supplied[] = {limits.host_bytes, limits.member_bytes, limits.metadata_bytes,
        limits.groups, limits.group_members, limits.blocks};
    const std::size_t allowed[] = {maximum.host_bytes, maximum.member_bytes, maximum.metadata_bytes,
        maximum.groups, maximum.group_members, maximum.blocks};
    for (unsigned i = 0; i < std::size(supplied); ++i)
        Require(supplied[i] && supplied[i] <= allowed[i], "Invalid auxiliary limits");
    Require(member_bytes && member_bytes <= limits.member_bytes && source.canonical_nodes <= UINT32_MAX,
            "Auxiliary member/count cap exceeded");
    std::size_t total = sizeof(AuxiliaryData)+sizeof(source::CanonicalData)+256;
    // Retained canonical backing/declaration are charged once. The supplied
    // auxiliary member, parser, decoded nodes, indices and draft coexist.
    Add(total, declaration.owned_payload_bytes, 1, limits.host_bytes);
    for (const auto& array : source.arrays) Add(total, array.bytes.capacity()+1, 1, limits.host_bytes);
    Add(total, source.parts.capacity(), sizeof(source::PartDeclaration), limits.host_bytes);
    Add(total, source.selected_parts.capacity()+source.excluded_parts.capacity(), sizeof(SourceId), limits.host_bytes);
    Add(total, source.canonical_bytes.size(), 8, limits.host_bytes);
    Add(total, source.scope_bytes.capacity()+1, 1, limits.host_bytes);
    Add(total, member_bytes, 1, limits.host_bytes);
    Add(total, limits.metadata_bytes, 6, limits.host_bytes);
    Add(total, source.canonical_nodes, 40, limits.host_bytes);
    Add(total, limits.blocks, sizeof(SourceEvidence)+sizeof(UnresolvedBlock)+256, limits.host_bytes);
    Add(total, limits.groups, sizeof(AuxiliaryGroup), limits.host_bytes);
    Add(total, limits.group_members, sizeof(AuxiliaryMemberNode)+3*sizeof(SourceId)+256, limits.host_bytes);
    return total;
}
std::size_t OwnedPayload(const AuxiliaryData& d, std::size_t cap) {
    std::size_t total = sizeof(AuxiliaryData);
    const auto text = [&](const std::string& s) { Add(total, s.capacity()+1, 1, cap); };
    const auto vector = [&](const auto& v) {
        Add(total, v.capacity(), sizeof(typename std::decay_t<decltype(v)>::value_type), cap);
    };
    text(d.filename);
    text(d.member_sha256);
    vector(d.sources);
    vector(d.groups);
    vector(d.member_nodes);
    vector(d.original_walls);
    vector(d.wall_source_files);
    vector(d.other_unresolved_constraints);
    for (const auto& s : d.sources) {
        text(s.block.filename); text(s.block.keyword); text(s.block.sha256); text(s.block.raw_text);
        vector(s.cards);
        for (const auto& card : s.cards) text(card.second);
    }
    for (const auto& group : d.groups) {
        vector(group.evidence.source_nodes);
        vector(group.evidence.master_nodes);
        vector(group.evidence.slave_nodes);
    }
    for (const auto* blocks : {&d.original_walls, &d.other_unresolved_constraints})
        for (const auto& b : *blocks) { text(b.filename); text(b.keyword); text(b.sha256); }
    for (const auto& file : d.wall_source_files) { text(file.filename); text(file.sha256); }
    return total;
}
} // namespace crash::modelio::tied_shell::auxiliary_detail
