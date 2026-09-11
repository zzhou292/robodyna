#include "Internal.h"
#include "modelio/physical_scope/CanonicalDomain.h"
#include "output/BoundedArrayIO.h"

namespace crash::modelio::physical_domain {
struct VehiclePhysicalDomain::Storage {
    explicit Storage(const physical_scope::PhysicalScope& value) : source(value) {}
    physical_scope::PhysicalScope source;
    tl::fea::NodalNodeDomain domain;
    tl::fea::rigid::NodalRigidPartTopology topology;
    std::vector<GroupSelection> groups;
    Counts counts;
    Forecast forecast;
};
VehiclePhysicalDomain VehiclePhysicalDomain::Prepare(const physical_scope::PhysicalScope& source, Policy policy, Limits limits) {
    const auto forecast = Preflight(source, policy, limits);
    auto selected = detail::Select(source.data().plain_groups, source.data().point_masses);
    const auto& canonical = source.tied_source().canonical().data();
    const auto& id_array = physical_scope::source::FindArray(canonical, "node_ids");
    const auto& xyz_array = physical_scope::source::FindArray(canonical, "node_positions");
    const auto ids = output::arrays::Decode<std::uint64_t>(id_array.descriptor, id_array.bytes);
    const auto xyz = output::arrays::Decode<double>(xyz_array.descriptor, xyz_array.bytes);
    output::Require(ids.size() <= SIZE_MAX / 3 && xyz.size() == 3 * ids.size() && source.data().node_roles.size() == ids.size(),
                    "Canonical physical domain coordinate extent changed");
    std::vector<tl::fea::NodalDomainNode> nodes;
    nodes.reserve(source.data().counts.with_type25_nodes + source.data().point_masses.size());
    for (std::size_t n = 0; n < ids.size(); ++n)
        if ((source.data().node_roles[n] & (physical_scope::PhysicalRoles | physical_scope::ProvisionalType25)) ||
            selected.point_nodes.count(ids[n]))
            nodes.push_back({ids[n], {xyz[3*n], xyz[3*n+1], xyz[3*n+2]}});
    auto next = std::make_shared<Storage>(source);
    auto domain_limits = tl::fea::NodalDomainLimits::Vehicle();
    domain_limits.max_host_bytes = limits.domain_bytes;
    const auto& original = source.point_mass_source().rigid_source().topology();
    const auto domain_report = next->domain.Initialize({original.source_instance_id(), nodes.data(), nodes.size()}, domain_limits);
    output::Require(bool(domain_report), domain_report.message);
    physical_scope::detail::CheckDomain(ids, xyz, source.data().node_roles, next->domain,
        original.source_instance_id(), source.data().counts.with_type25_nodes);
    detail::PrepareTopology(original, selected, limits.topology_bytes, next->topology);
    for (std::size_t n = 0; n < next->topology.member_count(); ++n)
        output::Require(next->domain.Find(next->topology.original_members()[n]) != SIZE_MAX,
                        "Original PART member is absent from the declared case domain");
    for (const auto& group : selected.groups)
        for (const auto nid : group.members)
            output::Require(next->domain.Find(nid) != SIZE_MAX, "Retained plain member is absent from the case domain");
    next->counts = selected.counts;
    next->groups = std::move(selected.groups);
    next->forecast = forecast;
    return VehiclePhysicalDomain(std::move(next));
}
const physical_scope::PhysicalScope& VehiclePhysicalDomain::source() const noexcept { return storage_->source; }
const tl::fea::NodalNodeDomain& VehiclePhysicalDomain::domain() const noexcept { return storage_->domain; }
const tl::fea::rigid::NodalRigidPartTopology& VehiclePhysicalDomain::topology() const noexcept { return storage_->topology; }
const std::vector<GroupSelection>& VehiclePhysicalDomain::plain_groups() const noexcept { return storage_->groups; }
Counts VehiclePhysicalDomain::counts() const noexcept { return storage_->counts; }
const Forecast& VehiclePhysicalDomain::forecast() const noexcept { return storage_->forecast; }
} // namespace crash::modelio::physical_domain
