#include "Internal.h"
#include "SourcePolicy.h"
#include <algorithm>

namespace crash::modelio::type45 {
namespace detail {
void Add(std::size_t& bytes, std::size_t count, std::size_t width, std::size_t cap) {
    Require(width && bytes <= cap && count <= (cap - bytes) / width, "Complete TYPE45 source budget exceeds cap");
    bytes += count * width;
}
} // namespace detail
Forecast VehicleType45Source::Preflight(const physical_domain::VehiclePhysicalDomain& input, Policy policy, Limits limits) {
    using namespace detail;
    const Limits hard;
    Require(input.policy() == DomainPolicy(policy), "Original TYPE45 policy differs from its physical source domain");
    Require(limits.host_bytes && limits.host_bytes <= hard.host_bytes &&
        limits.rows && limits.rows <= hard.rows && limits.metadata_bytes && limits.metadata_bytes <= hard.metadata_bytes,
        "Invalid original TYPE45 source policy/caps");
    const auto& source = input.source();
    const auto& rigid = source.point_mass_source().rigid_source();
    const auto& canonical = source.tied_source().canonical().data();
    const auto& units = canonical.inputs.units;
    Require(units.length_to_m == .001 && units.mass_to_kg == 1000 && units.time_to_s == 1,
            "Original TYPE45 source units changed");
    Require(input.domain().prepared() && input.domain().source_instance_id() == rigid.topology().source_instance_id() &&
        &rigid.source().canonical().data() == &canonical, "Original TYPE45 source/domain identity changed");
    std::size_t rows = 0, metadata = 0;
    for (const auto& block : rigid.data().sources) if (IsJoint(block.block.keyword)) {
        Require(++rows <= limits.rows, "Joint row count exceeds cap");
        Add(metadata, block.block.raw_text.size(), 1, limits.metadata_bytes);
    }
    Require(rows == 44, "Original TYPE45 requires all 44 declarations");
    Forecast result;
    result.previous_phase = input.forecast().total_bytes;
    // Retained source includes every immutable upstream handle. Decode/scratch
    // in the physical-domain construction are retired; actual domain and
    // topology backing plus conservative group reservation remain owned.
    result.retained_source = input.forecast().retained_source;
    for (auto bytes : {input.domain().owned_payload_bytes(), input.topology().owned_payload_bytes(),
                       input.forecast().selection_bytes}) Add(result.retained_source, bytes, 1, limits.host_bytes);
    Add(result.decode_bytes, canonical.canonical_nodes, 2 * (sizeof(std::uint64_t) + 3*sizeof(double)), limits.host_bytes);
    Add(result.mapping_bytes, 32768, sizeof(BodyMember), limits.host_bytes);
    Add(result.mapping_bytes, limits.rows, sizeof(Row), limits.host_bytes); // Atomic map scratch.
    Add(result.mapping_bytes, metadata, 2, limits.host_bytes); // One bounded card decode and string stream.
    result.result_bytes = sizeof(VehicleType45Source) + sizeof(Data) + sizeof(physical_domain::VehiclePhysicalDomain) + 1024;
    Add(result.result_bytes, limits.rows, sizeof(Row), limits.host_bytes);
    for (auto bytes : {result.retained_source, result.decode_bytes, result.mapping_bytes, result.result_bytes})
        Add(result.current_phase, bytes, 1, limits.host_bytes);
    result.total_bytes = std::max(result.previous_phase, result.current_phase);
    Require(result.total_bytes <= limits.host_bytes, "Prior complete TYPE45 source phase exceeds cap");
    return result;
}
} // namespace crash::modelio::type45
