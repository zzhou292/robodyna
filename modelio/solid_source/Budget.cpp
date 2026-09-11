#include "Internal.h"
#include <iterator>
#include <type_traits>

namespace crash::modelio::solid_source::detail {
namespace {
void Add(std::size_t& bytes, std::size_t count, std::size_t width, std::size_t cap) {
    Require(width && bytes <= cap && count <= (cap - bytes) / width,
            "Solid source host byte cap exceeded");
    bytes += count * width;
}
}
void CheckOriginal(const source::CanonicalData& source) {
    Require(source.archive_sha256 == "aff8194c456726a678d6cc11f644316ca70f3d9b37c4db622726b7b2985b0451" &&
        source.inputs.canonical_manifest.sha256 == "c82f1886b8935d69ff7db4c29c700370e3a057579fab80d02664a253bc7af1c8" &&
        source.inputs.source_member.sha256 == "67208317e6c8eb1dd43b80001508915ccaace7bc0a745e1aa5a3b33f394df301" &&
        source.inputs.source_member.bytes == 42846753 && source.canonical_nodes == 393165,
        "Original solid source authority changed");
    const auto& units = source.inputs.units;
    Require(units.mass == "t" && units.length == "mm" && units.time == "s" &&
        units.mass_to_kg == 1000 && units.length_to_m == .001 && units.time_to_s == 1,
        "Original solid source units changed");
}
Forecast Budget(const source::CanonicalData& source, Policy policy, Limits limits) {
    const Limits hard;
    const std::size_t value[]{limits.host_bytes, limits.member_bytes, limits.metadata_bytes,
        limits.parents, limits.nodes, limits.source_solids, limits.blocks};
    const std::size_t maximum[]{hard.host_bytes, hard.member_bytes, hard.metadata_bytes,
        hard.parents, hard.nodes, hard.source_solids, hard.blocks};
    for (unsigned i = 0; i < std::size(value); ++i)
        Require(value[i] && value[i] <= maximum[i], "Invalid solid source limits");
    const auto census = ExpectedCensus(policy);
    CheckOriginal(source);
    const auto& records = source::FindArray(source, "solids_records");
    Require(records.descriptor.layout.columns == 10 &&
        records.descriptor.layout.rows <= limits.source_solids && source.canonical_nodes <= limits.nodes &&
        source.inputs.source_member.bytes <= limits.member_bytes && limits.parents >= census.parents,
        "Original solid source count exceeds capacity");
    Forecast result;
    result.fixed_bytes = sizeof(Data) + sizeof(source::CanonicalData) + sizeof(VehicleSolidSource) + 512;
    for (const auto& array : source.arrays)
        Add(result.canonical_bytes, array.bytes.capacity() + 1, 1, limits.host_bytes);
    Add(result.canonical_bytes, source.parts.capacity(), sizeof(source::PartDeclaration), limits.host_bytes);
    Add(result.canonical_bytes, source.selected_parts.capacity() + source.excluded_parts.capacity(),
        sizeof(std::uint64_t), limits.host_bytes);
    // Retained metadata plus two bounded DOMs; member bytes coexist with both.
    Add(result.parsing_bytes, source.canonical_bytes.size(), 7, limits.host_bytes);
    Add(result.parsing_bytes, source.scope_bytes.size(), 7, limits.host_bytes);
    Add(result.parsing_bytes, source.inputs.source_member.bytes, 1, limits.host_bytes);
    Add(result.parsing_bytes, limits.metadata_bytes, 4, limits.host_bytes);
    Add(result.parsing_bytes, limits.blocks, 256, limits.host_bytes);
    // Full canonical decoded node/solid fields, decoder temporaries, source-line
    // requests and all selected raw cards. No retained geometry JSON is created.
    Add(result.geometry_bytes, source.canonical_nodes, 80, limits.host_bytes);
    Add(result.geometry_bytes, records.descriptor.layout.rows, 192, limits.host_bytes);
    Add(result.geometry_bytes, limits.parents, sizeof(Row) + 256, limits.host_bytes);
    Add(result.reference_bytes, limits.parents, sizeof(tl::fea::solid18::Reference) +
        sizeof(tl::fea::solid24::Reference) + sizeof(tl::fea::solid6z::Reference) +
        (census.solid18_law44 ? sizeof(tl::fea::solid18::law44::Reference) : 0), limits.host_bytes);
    for (auto bytes : {result.fixed_bytes, result.canonical_bytes, result.parsing_bytes,
                      result.geometry_bytes, result.reference_bytes})
        Add(result.total_bytes, bytes, 1, limits.host_bytes);
    return result;
}
std::size_t OwnedPayload(const Data& data, Limits limits) {
    std::size_t bytes = sizeof(Data) + sizeof(VehicleSolidSource) + 256;
    const auto vector = [&](const auto& values) {
        Add(bytes, values.capacity(), sizeof(typename std::decay_t<decltype(values)>::value_type), limits.host_bytes);
    };
    const auto text = [&](const auto& value) { Add(bytes, value.capacity() + 1, 1, limits.host_bytes); };
    vector(data.parts);
    vector(data.rows);
    vector(data.canonical_nodes);
    vector(data.plastic_strain);
    vector(data.yield_stress_pa);
    vector(data.solid18);
    vector(data.solid24);
    vector(data.solid6z);
    vector(data.solid18_law44);
    vector(data.rear_plastic_strain);
    vector(data.rear_yield_stress_pa);
    vector(data.sources);
    for (const auto& row : data.rows) text(row.raw_card);
    for (const auto& source : data.sources) {
        text(source.block.filename);
        text(source.block.keyword);
        text(source.block.raw_text);
        text(source.block.sha256);
        vector(source.cards);
        for (const auto& card : source.cards) text(card.second);
    }
    return bytes; // Owned payload plus control reserve; excludes allocator/RSS.
}
} // namespace crash::modelio::solid_source::detail
