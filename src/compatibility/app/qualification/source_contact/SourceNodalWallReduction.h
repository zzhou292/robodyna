#pragma once
#include "SourceNodalWallFixture.h"

namespace crash::qualification::source_contact::nodal::reduction {
// Qualification GPU composition only. This does not copy a contact law or
// replace owning TL host reduction: tests compare every certificate/identity
// with that independently compiled operation. Same sorted-parent/native-node
// order makes each share enter a unique node exactly once.
TL_SURFACE_HD inline bool Sum(sc::Q4CertifiedIntegral& a,const sc::Q4CertifiedIntegral& b) {
    sc::Q4IntegralInterval truth;
    return sc::q4_bounds::Add({a.lower,a.upper},{b.lower,b.upper},&truth) &&
        sc::q4_bounds::Certify(a.value+b.value,truth,&a);
}
TL_SURFACE_HD inline bool AddShare(sc::NodalWallPointResult& node,const sc::NodalWallPointResult& share) {
    if (!node.valid) { node=share; return true; }
    if (node.node!=share.node || node.fixed!=share.fixed || node.base_epoch!=share.base_epoch ||
        node.attempt!=share.attempt || !Sum(node.force,share.force) || !Sum(node.potential,share.potential) ||
        !Sum(node.stiffness,share.stiffness)) return false;
    return node.fixed || (node.row.valid && share.row.valid && node.row.count==1 && share.row.count==1 &&
        sc::q4_bounds::AddScalar(node.row.stiffness[0],share.row.stiffness[0],true,&node.row.stiffness[0]));
}
TL_SURFACE_HD inline bool CompleteNode(sc::NodalWallPointResult& node,sc::Vec3 velocity) {
    node.force_world={-node.force.value,0,0}; node.wall_reaction={node.force.value,0,0};
    node.wall_moment=sc::geometry_detail::Cross(node.wall_point,node.wall_reaction);
    node.surface_power=sc::Dot(node.force_world,velocity);
    node.local_velocity_first_timestep=0;
    return node.valid && sc::IsFinite(node.wall_moment) && sc::IsFinite(node.surface_power);
}
TL_SURFACE_HD inline bool AddNode(sc::NodalWallResult& out,const sc::NodalWallPointResult& node) {
    if (!Sum(out.resultant,node.force) || !Sum(out.potential,node.potential)) return false;
    out.wall_reaction=sc::Add(out.wall_reaction,node.wall_reaction);
    out.wall_moment=sc::Add(out.wall_moment,node.wall_moment); out.surface_power+=node.surface_power;
    return sc::IsFinite(out.wall_reaction) && sc::IsFinite(out.wall_moment) && sc::IsFinite(out.surface_power);
}
} // namespace crash::qualification::source_contact::nodal::reduction
