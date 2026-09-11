#include "VehiclePhysicalDomain.h"
#include "output/ArtifactIO.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>

namespace crash::modelio::physical_domain {
Forecast VehiclePhysicalDomain::Preflight(const physical_scope::PhysicalScope& source, Policy policy, Limits limits) {
    const Limits hard;
    output::Require(policy == Policy::RetainedShellAssembliesV1 && limits.host_bytes && limits.host_bytes <= hard.host_bytes &&
        limits.domain_bytes && limits.domain_bytes <= hard.domain_bytes && limits.topology_bytes &&
        limits.topology_bytes <= hard.topology_bytes, "Invalid physical domain source policy or limits");
    const auto& data = source.data();
    output::Require(data.plain_groups.size() == 759 && data.part_roots.size() == 20 && data.point_masses.size() == 155 &&
        data.counts.with_type25_nodes <= 524288 - data.point_masses.size(), "Original physical domain source census changed");
    Forecast result;
    result.previous_phase = source.forecast().total_bytes;
    tl::util::BoundedArenaLayout retained(limits.host_bytes), decoding(limits.host_bytes), selection(limits.host_bytes),
        phase(limits.host_bytes);
    tl::util::ArenaRegion ignored;
    output::Require(retained.Append<unsigned char>(source.forecast().source_reservation, ignored) &&
        retained.Append<unsigned char>(source.forecast().additional_retained, ignored) &&
        retained.Append<unsigned char>(data.owned_payload_bytes, ignored), "Physical domain retained source exceeds cap");
    result.retained_source = retained.bytes();
    const auto nodes = source.tied_source().canonical().data().canonical_nodes;
    output::Require(decoding.Append<std::uint64_t>(nodes, ignored) && decoding.Append<std::uint64_t>(nodes, ignored) &&
        decoding.Append<std::array<double,3>>(nodes, ignored) && decoding.Append<std::array<double,3>>(nodes, ignored),
        "Physical domain coordinate decoding exceeds cap");
    result.decode_bytes = decoding.bytes();
    // Bound retained inclusion/exclusion vectors, tree nodes, and topology input
    // spans before selection. All original plain members together fit32768.
    output::Require(selection.Append<GroupSelection>(data.plain_groups.size(), ignored) &&
        selection.Append<std::uint64_t>(4 * 32768, ignored) &&
        selection.Append<unsigned char>(2 * 155 * 128, ignored) &&
        selection.Append<unsigned char>(1024 * 128, ignored), "Physical source selection exceeds cap");
    result.selection_bytes = selection.bytes();
    result.domain_reservation = limits.domain_bytes;
    result.topology_reservation = limits.topology_bytes;
    output::Require(phase.Append<unsigned char>(sizeof(VehiclePhysicalDomain) + 2048, ignored) &&
        phase.Append<unsigned char>(result.retained_source, ignored) &&
        phase.Append<unsigned char>(result.decode_bytes, ignored) &&
        phase.Append<unsigned char>(result.selection_bytes, ignored) &&
        phase.Append<unsigned char>(result.domain_reservation, ignored) &&
        phase.Append<unsigned char>(result.topology_reservation, ignored) &&
        phase.Append<tl::fea::NodalDomainNode>(data.counts.with_type25_nodes + data.point_masses.size(), ignored),
        "Complete physical domain source construction exceeds cap");
    result.current_phase = phase.bytes();
    result.total_bytes = std::max(result.previous_phase, result.current_phase);
    output::Require(result.total_bytes <= limits.host_bytes, "Prior physical source phase exceeds domain cap");
    return result;
}
} // namespace crash::modelio::physical_domain
