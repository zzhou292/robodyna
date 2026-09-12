#include "Packing.h"
#include "output/ArtifactIO.h"
#include "lib_utils/BoundedArena.h"
#include <cmath>
#include <cstring>
namespace crash::cases::vehicle_runtime::detail {
namespace {
bool Bits(double a,double b) noexcept { return std::memcmp(&a,&b,sizeof(double)) == 0; }
}
tl::fea::HostNodalKinematicsView OwnerPacking::kinematics() const noexcept {
    return {position.data(),velocity.data(),spin.data(),mass.size(),orientation.data()};
}
tl::fea::NodalDofConfig OwnerPacking::dofs() const noexcept {
    return {fixed.data(),rotation_fixed.data(),inverse_inertia.data(),rotation_present.data()};
}
std::size_t OwnerPacking::capacity_bytes() const noexcept {
    return sizeof(double) * (position.capacity()+velocity.capacity()+spin.capacity()+orientation.capacity()+
        mass.capacity()+inertia.capacity()+inverse_mass.capacity()+inverse_inertia.capacity()) +
        fixed.capacity()+rotation_fixed.capacity()+rotation_present.capacity();
}
std::size_t PackingBytes(std::size_t nodes,std::size_t cap) {
    output::Require(nodes && nodes <= tl::fea::MaxActiveNodalStateNodes,"Invalid owner packing extent");
    tl::util::BoundedArenaLayout budget(cap);
    tl::util::ArenaRegion unused;
    output::Require(budget.Append<double>(17*nodes,unused) && budget.Append<std::uint8_t>(3*nodes,unused),
                    "Complete owner packing exceeds host budget");
    return budget.bytes();
}
OwnerPacking PackOwner(const tl::fea::NodalCoefficientLedger& ledger,
    const tl::fea::NodalRigidAssemblyBinding& rigid,const SourceRoles& roles,
    tl::math::Vec3 velocity,std::size_t cap) {
    using output::Require;
    const auto count = ledger.nodes().size();
    const auto forecast = PackingBytes(count,cap);
    Require(ledger.prepared() && rigid.prepared() && rigid.coefficients()->Matches(ledger) &&
        roles.node.size() == count && std::isfinite(velocity.x) &&
        std::isfinite(velocity.y) && std::isfinite(velocity.z),"Incomplete owner coefficient/role packing scope");
    OwnerPacking next;
    next.position.resize(3*count);
    next.velocity.resize(3*count);
    next.spin.resize(3*count);
    next.orientation.resize(4*count);
    next.mass.resize(count);
    next.inertia.resize(count);
    next.inverse_mass.resize(count);
    next.inverse_inertia.resize(count);
    next.fixed.resize(count);
    next.rotation_fixed.resize(count);
    next.rotation_present.resize(count);
    Require(next.capacity_bytes() <= forecast,"Actual owner packing capacities exceed preflight");
    for (std::size_t node = 0; node < count; ++node) {
        const auto flags = roles.node[node];
        Require(bool(flags & Beam18Endpoint) == bool(ledger.nodes()[node].occurrences.beam18),
                "Structural beam role differs from actual endpoint coefficients");
        const auto* member = rigid.FindMember(node);
        Require(bool(flags & (Part|PlainRigid)) == bool(member),"Rigid role differs from actual prepared membership");
        const auto& value = ledger.nodes()[node].coefficients;
        const auto& x = ledger.domain()->nodes()[node].position;
        next.position[3*node] = x.x;
        next.position[3*node+1] = x.y;
        next.position[3*node+2] = x.z;
        next.velocity[3*node] = velocity.x;
        next.velocity[3*node+1] = velocity.y;
        next.velocity[3*node+2] = velocity.z;
        next.orientation[4*node] = 1;
        next.mass[node] = value.mass;
        next.inertia[node] = value.isotropic_inertia;
        const bool dependent = flags & CinSecondary;
        const bool present = flags != 0;
        Require(std::isfinite(value.mass) && value.mass >= 0 &&
            std::isfinite(value.isotropic_inertia) && value.isotropic_inertia >= 0,
            "Native owner coefficient is nonfinite or negative");
        Require(!dependent || (!member && !(flags & CinMaster)),"CIN hierarchy/rigid intersection is unsupported");
        Require(dependent || member || value.mass > 0,"Ordinary free translation has no source mass");
        Require(present ? (dependent || member || value.isotropic_inertia > 0) : value.isotropic_inertia == 0,
            "Source rotational role and native inertia disagree");
        if (member) Require(Bits(member->mass_kg,value.mass) &&
            Bits(member->isotropic_inertia_kg_m2,value.isotropic_inertia),"Prepared rigid coefficient bits differ");
        next.rotation_present[node] = present;
        next.inverse_mass[node] = dependent || value.mass == 0 ? 0 : 1/value.mass;
        next.inverse_inertia[node] = dependent || !present || value.isotropic_inertia == 0 ? 0 : 1/value.isotropic_inertia;
        Require(std::isfinite(next.inverse_mass[node]) && std::isfinite(next.inverse_inertia[node]),
            "Native source reciprocal exceeds finite owner domain");
    }
    return next;
}
} // namespace crash::cases::vehicle_runtime::detail
