#include "SourceAssemblyWallSetup.h"
#include "lib_src/solvers/NodalNativePhysicalCoefficients.h"
#include <cmath>

namespace crash::cases::source_assembly {
SourceAssemblyWallReport SourceAssemblyWallSetup::MakeDeviceConfig(const tl::fea::NodalStamp& stamp,
    tlfea::contact::NodalWallDeviceConfig* output,const SourceAssemblyWallDeviceLimits& limits) const {
    using Code=SourceAssemblyWallStatus;namespace sc=tlfea::contact;namespace fe=tl::fea;
    if(!initialized())return {Code::NotInitialized,"Assembly wall setup is not prepared"};
    if(!output||!fe::trial_identity::Disjoint(output,sizeof(*output),&stamp,sizeof(stamp))||
       !fe::trial_identity::Disjoint(output,sizeof(*output),&limits,sizeof(limits))||
       !fe::trial_identity::Disjoint(output,sizeof(*output),this,sizeof(*this)))
        return {Code::InvalidInput,"Contact configuration output is missing or aliases its inputs"};
    const auto& b=*bindings();const auto* groups=b.rigid_groups();const auto* weights=source_geometry()->weights();
    const fe::NodalRigidGroupInfo expected{b.source_instance_id(),groups?groups->group_count():0,groups?groups->member_count():0};
    if(!groups||!fe::native_physical_coefficients::ValidScope(expected,b.shells().node_count())||
       !fe::SameRigidGroupInfo(expected,stamp.rigid_groups)||!stamp.owner_id||stamp.node_count!=b.shells().node_count()||
       !stamp.has_rotations||stamp.temporal_scheme!=fe::NodalTemporalScheme::StaggeredHalfKickStart||
       stamp.velocity_phase!=fe::NodalVelocityPhase::Collocated||stamp.epoch||stamp.time!=0||stamp.velocity_time!=0||
       stamp.reactions_valid||stamp.reaction_base_epoch||stamp.reaction_time!=0||stamp.reaction_kick_dt!=0||
       !std::isfinite(stamp.fixed_dt)||stamp.fixed_dt<1e-12)
        return {Code::ScopeMismatch,"Declared fresh owner stamp differs from the complete assembly group scope"};
    const auto& counts=limits.counts;
    if(!counts.parents||counts.parents>sc::MaxActiveNodalWallDeviceParents||counts.parents<weights->parent_count()||
       !counts.nodes||counts.nodes>sc::MaxActiveNodalWallDeviceNodes||counts.nodes<weights->node_count()||
       !counts.global_nodes||counts.global_nodes>sc::MaxActiveNodalWallDeviceNodes||counts.global_nodes<stamp.node_count||
       !limits.max_device_bytes||limits.max_device_bytes>sc::MaxActiveNodalWallDeviceBytes||
       !limits.max_host_bytes||limits.max_host_bytes>sc::MaxNodalWallHostBytes)
        return {Code::ResourceLimit,"Declared contact device limits do not cover the complete source scope"};
    const auto& s=*settings();sc::NodalWallDeviceConfig next;
    next.owner=stamp;next.configuration_id=s.configuration_id;next.qualification_id=s.qualification_id;
    next.wall_binding_id=s.wall_binding_id;next.limits=counts;next.max_device_bytes=limits.max_device_bytes;
    next.max_host_bytes=limits.max_host_bytes;next.exposed_clearance=s.exposed_clearance;
    next.law={placed_wall()->geometry()->wall_x(),certificate()->penalty.stiffness_per_area,
              s.penalty.penetration_cap,s.parent_force_error,s.parent_energy_error};
    *output=next;return {Code::Ok,"Complete contact configuration declared; actual owner and allocation checks remain in TL"};
}
} // namespace crash::cases::source_assembly
