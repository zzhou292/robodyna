#include "Internal.h"

namespace crash::cases::vehicle_startup::physical_model::detail {
fe::NodalRigidGroupMember PlainMember(const fe::NodalNodeDomain& domain, const fe::NodalCoefficientLedger& ledger,
                                     std::uint64_t nid) {
    const auto n = domain.Find(nid);
    Require(n != SIZE_MAX, "Plain group source NID is absent from the complete physical domain");
    const auto& row = ledger.nodes()[n];
    Require(fe::HasCoefficientProducer(row.occurrences), "Plain group has no real coefficient producer");
    const auto& c = row.coefficients;
    // These are diagnostic partitions, never reconstructed authoritative J.
    // Solid and ELEMENT_MASS producers contribute no scalar rotational inertia.
    const double physical = (c.shell.physical_inertia + c.type25.isotropic_inertia) +
                            (c.type13.isotropic_inertia - c.type13.added_inertia);
    const double added = c.shell.added_inertia + c.type13.added_inertia;
    return {nid, n, domain.nodes()[n].position, c.mass, c.isotropic_inertia, physical, added, c.beam18.isotropic_inertia};
}
void PreparePlain(const modelio::physical_domain::VehiclePhysicalDomain& selection,
                  const fe::NodalCoefficientLedger& ledger, std::size_t cap, fe::NodalRigidGroupModel& model) {
    PreparePlain(selection, selection.domain(), ledger, cap, model);
}
void PreparePlain(const modelio::physical_domain::VehiclePhysicalDomain& selection,
                  const fe::NodalNodeDomain& domain, const fe::NodalCoefficientLedger& ledger,
                  std::size_t cap, fe::NodalRigidGroupModel& model) {
    std::vector<fe::NodalRigidGroupMember> members;
    std::vector<fe::NodalRigidGroupInput> groups;
    members.reserve(selection.counts().plain_members);
    groups.reserve(selection.plain_groups().size());
    for (const auto& selected : selection.plain_groups()) {
        if (selected.disposition == modelio::physical_domain::GroupDisposition::Omitted) continue;
        const auto& source = selection.source().data().plain_groups.at(selected.source_group);
        const auto first = members.size();
        for (const auto nid : selected.members) members.push_back(PlainMember(domain, ledger, nid));
        groups.push_back({source.id, selected.case_node_set_id, members.data() + first, selected.members.size()});
    }
    Require(members.size() == selection.counts().plain_members, "Physical plain member inventory changed");
    auto limits = fe::NodalRigidGroupLimits::Vehicle(); limits.max_host_bytes = cap;
    const fe::NodalRigidGroupModelInput input{domain.source_instance_id(), domain.node_count(),
        groups.data(), groups.size(), {1000, .001}, limits};
    const auto report = modelio::physical_scope::HasVehicleSupports(selection.source().solid_source().data().policy)
        ? model.InitializeNativeTotal(input) : model.InitializePhysical(input);
    if (!report) throw std::runtime_error("Original plain rigid group " + std::to_string(report.group) +
                                         " rejected: " + report.message);
}
} // namespace crash::cases::vehicle_startup::physical_model::detail
