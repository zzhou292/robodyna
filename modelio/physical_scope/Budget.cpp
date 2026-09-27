#include "Internal.h"
#include "SourcePolicy.h"
#include <type_traits>

namespace crash::modelio::physical_scope::detail {
void Add(std::size_t& bytes, std::size_t count, std::size_t width, std::size_t cap) {
    Require(width && bytes <= cap && count <= (cap - bytes) / width, "Physical scope byte cap exceeded");
    bytes += count * width;
}
std::size_t OwnedPayload(const Data& data, Limits limits) {
    std::size_t bytes = sizeof(Data) + sizeof(PhysicalScope) + 512;
    const auto vector = [&](const auto& values) {
        Add(bytes, values.capacity(), sizeof(typename std::decay_t<decltype(values)>::value_type), limits.host_bytes);
    };
    vector(data.node_roles);
    vector(data.plain_groups);
    vector(data.part_roots);
    vector(data.spotwelds);
    vector(data.evidence);
    vector(data.point_masses);
    vector(data.rigid_skin);
    for (const auto* groups : {&data.plain_groups, &data.part_roots})
        for (const auto& group : *groups) vector(group.members);
    for (const auto& node : data.evidence) vector(node.incidence);
    return bytes;
}
} // namespace crash::modelio::physical_scope::detail

namespace crash::modelio::physical_scope {
Forecast PhysicalScope::PreflightImpl(const rigid::point_mass::Source& masses,
    const tied_shell::TiedShellDeclaration& tied, const type13::SourceType13& beams,
    const solid_source::VehicleSolidSource& solids, const beam18::Source* structural, Limits limits) {
    using namespace detail;
    const Limits hard;
    const std::size_t values[]{limits.host_bytes, limits.nodes, limits.groups, limits.members,
        limits.spotwelds, limits.evidence_nodes, limits.incidences};
    const std::size_t maximum[]{hard.host_bytes, hard.nodes, hard.groups, hard.members,
        hard.spotwelds, hard.evidence_nodes, hard.incidences};
    for (unsigned i = 0; i < std::size(values); ++i)
        Require(values[i] && values[i] <= maximum[i], "Invalid physical source census limits");
    const auto& canonical = masses.rigid_source().source().canonical().data();
    Require(&canonical == &tied.canonical().data() && &canonical == &solids.canonical().data() &&
        beams.data().canonical_manifest_sha256 == canonical.inputs.canonical_manifest.sha256,
        "Physical source inputs do not share the authenticated canonical authority");
    Require(canonical.inputs.tire_policy == "omit_original_tire_shells" && canonical.excluded_parts.size() == 8 &&
        canonical.canonical_nodes <= limits.nodes && tied.data().groups.size() <= limits.groups,
        "Physical source selection or count exceeds the explicit original profile");
    Require(masses.rigid_source().topology().member_count() <= limits.members &&
        masses.rigid_source().topology().other_rigid_member_count() <=
            limits.members - masses.rigid_source().topology().member_count(),
        "Physical source complete group members exceed cap");
    const bool supports=HasVehicleSupports(solids.data().policy);
    Require(supports==bool(structural),"Complete support source requires declared support solids and structural beams");
    if(structural) Require(&structural->canonical().data()==&canonical &&
        structural->data().policy==beam18::Policy::OriginalCircularFourPointLaw44V1 &&
        structural->data().rows.size()==142 && structural->data().canonical_endpoints.size()==146 &&
        solids.data().rows.size()==4980,"Complete vehicle support source identity or census differs");
    Forecast result;
    result.source_reservation = masses.data().startup_budget_bytes;
    for (auto bytes : {tied.data().owned_payload_bytes, beams.data().owned_payload_bytes, solids.data().owned_payload_bytes})
        Add(result.additional_retained, bytes, 1, limits.host_bytes);
    if(structural) Add(result.additional_retained,structural->data().owned_payload_bytes,1,limits.host_bytes);
    // Canonical backing is already charged by the authenticated masses source;
    // only the beam source own payload is additional.
    // Decoded node IDs, the node->evidence index, and the largest record/index
    // pair with both decoder byte temporaries. Families are traversed serially.
    std::size_t record_workspace = 0;
    for (const char* name : {"shells_records", "beams_records", "solids_records"})
        record_workspace = std::max(record_workspace, source::FindArray(canonical, name).bytes.size());
    Add(result.workspace, record_workspace, 3, limits.host_bytes);
    Add(result.workspace, canonical.canonical_nodes, 12, limits.host_bytes);
    Add(result.workspace, limits.evidence_nodes + limits.members + limits.spotwelds * 2, 64, limits.host_bytes);
    Add(result.result_reservation, 1, sizeof(Data) + sizeof(PhysicalScope) + 512, limits.host_bytes);
    Add(result.result_reservation, canonical.canonical_nodes, sizeof(std::uint16_t), limits.host_bytes);
    Add(result.result_reservation, limits.groups * 2, sizeof(Group), limits.host_bytes);
    Add(result.result_reservation, limits.members, sizeof(Member), limits.host_bytes);
    Add(result.result_reservation, limits.spotwelds, sizeof(Spotweld), limits.host_bytes);
    Add(result.result_reservation, limits.evidence_nodes, sizeof(NodeEvidence), limits.host_bytes);
    Add(result.result_reservation, limits.incidences * 2, sizeof(Incidence), limits.host_bytes);
    Add(result.result_reservation, masses.data().records.size(), 2 * sizeof(PointMass), limits.host_bytes);
    Add(result.result_reservation, masses.rigid_source().data().shell_count, 2 * sizeof(RigidSkin), limits.host_bytes);
    for (auto bytes : {result.source_reservation, result.additional_retained, result.workspace, result.result_reservation})
        Add(result.total_bytes, bytes, 1, limits.host_bytes);
    return result;
}
Forecast PhysicalScope::Preflight(const rigid::point_mass::Source& mass,
    const tied_shell::TiedShellDeclaration& tied,const type13::SourceType13& type13,
    const solid_source::VehicleSolidSource& solids,Limits limits) {
    return PreflightImpl(mass,tied,type13,solids,nullptr,limits);
}
Forecast PhysicalScope::PreflightVehicleSupports(const rigid::point_mass::Source& mass,
    const tied_shell::TiedShellDeclaration& tied,const type13::SourceType13& type13,
    const solid_source::VehicleSolidSource& solids,const beam18::Source& beams,Limits limits) {
    return PreflightImpl(mass,tied,type13,solids,&beams,limits);
}
} // namespace crash::modelio::physical_scope
