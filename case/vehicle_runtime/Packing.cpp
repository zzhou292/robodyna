#include "Packing.h"
#include "output/ArtifactIO.h"
#include "lib_utils/BoundedArena.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
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
void ApplyConstrainedStartup(OwnerPacking& packing,const tl::fea::ShellBatchStartup& startup,
    tl::util::ConstView<std::uint8_t> translation_fixed,tl::util::ConstView<std::uint8_t> rotation_fixed) {
    using output::Require;
    namespace start=tl::fea::shell_startup_detail;
    const auto count=packing.mass.size();
    Require(startup.kind==tl::fea::ShellBatchStartupKind::ReferenceConstrainedUniformTranslation&&
        start::ValidStartup(startup,true,true)&&count&&count<=tl::fea::MaxActiveNodalStateNodes&&
        translation_fixed.size()==count&&rotation_fixed.size()==count&&
        translation_fixed.data()&&rotation_fixed.data()&&packing.velocity.size()==3*count&&
        packing.fixed.size()==count&&packing.rotation_fixed.size()==count&&packing.rotation_present.size()==count&&
        packing.inverse_mass.size()==count&&packing.inverse_inertia.size()==count,
        "Constrained startup packing shape or descriptor differs");
    const auto disjoint=[&](const std::uint8_t* input) {
        using tl::fea::trial_identity::Disjoint;
        return Disjoint(input,count,&packing,sizeof(packing))&&
            Disjoint(input,count,packing.velocity.data(),3*count*sizeof(double))&&
            Disjoint(input,count,packing.inverse_mass.data(),count*sizeof(double))&&
            Disjoint(input,count,packing.inverse_inertia.data(),count*sizeof(double))&&
            Disjoint(input,count,packing.fixed.data(),count)&&
            Disjoint(input,count,packing.rotation_fixed.data(),count);
    };
    Require(disjoint(translation_fixed.data())&&disjoint(rotation_fixed.data()),
        "Constrained mask input overlaps mutable owner packing");
    for(std::size_t node=0;node<count;++node) {
        Require(std::isfinite(packing.inverse_mass[node])&&packing.inverse_mass[node]>=0&&
            std::isfinite(packing.inverse_inertia[node])&&packing.inverse_inertia[node]>=0,
            "Constrained projection cannot repair invalid source reciprocals");
        Require(packing.fixed[node]<=7&&translation_fixed[node]<=7&&packing.rotation_fixed[node]<=1&&
            rotation_fixed[node]<=1&&packing.rotation_present[node]<=1,
            "Constrained startup has an invalid fixed/presence mask");
        const auto prior=start::ProjectVelocity(startup.uniform_velocity,packing.fixed[node]);
        Require(Bits(packing.velocity[3*node],prior.x)&&Bits(packing.velocity[3*node+1],prior.y)&&
            Bits(packing.velocity[3*node+2],prior.z)&&
            (!(packing.rotation_fixed[node]|rotation_fixed[node])||packing.rotation_present[node]),
            "Packing does not represent the declared uniform source before projection");
    }
    for(std::size_t node=0;node<count;++node) {
        packing.fixed[node]|=translation_fixed[node];
        packing.rotation_fixed[node]|=rotation_fixed[node];
        const auto velocity=start::ProjectVelocity(startup.uniform_velocity,packing.fixed[node]);
        packing.velocity[3*node]=velocity.x;packing.velocity[3*node+1]=velocity.y;packing.velocity[3*node+2]=velocity.z;
        if(packing.fixed[node]==7)packing.inverse_mass[node]=0;
        if(packing.rotation_fixed[node])packing.inverse_inertia[node]=0;
    }
}
} // namespace crash::cases::vehicle_runtime::detail
