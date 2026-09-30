#include "Internal.h"
#include <cmath>
namespace crash::cases::native_scene::dynamics_detail {
f::NodalStateConfig OwnerConfig(const PhysicalSource& source,const DynamicsConfig& c) {
    output::Require(c.configuration&&c.qualification&&std::isfinite(c.fixed_dt)&&c.fixed_dt>=1e-12&&
        c.fixed_dt<=source.declared().data().step_cap_s.value_or(c.fixed_dt)&&
        std::isfinite(c.maximum_rotation_increment)&&c.maximum_rotation_increment>0&&c.maximum_rotation_increment<=.2&&
        c.limits.host_bytes&&c.limits.host_bytes<=512u<<20&&c.limits.device_bytes&&c.limits.device_bytes<=128u<<20,
        "Native dynamics identity, timestep or resource profile is invalid");
    f::NodalStateConfig out;out.node_count=source.physical().domain()->node_count();out.fixed_dt=c.fixed_dt;
    out.minimum_dt=1e-12;out.timestep_safety=source.declared().data().nodal_scale;out.temporal_scheme=f::NodalTemporalScheme::StaggeredHalfKickStart;
    if(!source.rigid().explicitly_empty())out.rigid_limits=f::NodalRigidOwnerLimits::VehicleAssembly();
    // Both profiles retain physical raw M/J. No extra force-stage sweep buffer
    // is needed for contact's accepted-mass borrow.
    out.capture_force_stage_accelerations=false;return out;
}
f::NodalStamp DescriptiveStamp(const PhysicalSource& source,const DynamicsConfig& c) {
    f::NodalStamp s;s.owner_id=1;s.node_count=source.physical().domain()->node_count();s.fixed_dt=c.fixed_dt;
    s.has_rotations=true;s.has_rotation_presence=true;s.temporal_scheme=f::NodalTemporalScheme::StaggeredHalfKickStart;
    if(!source.rigid().explicitly_empty()) {
        const auto& rigid=source.rigid();const auto* parts=rigid.parts();
        output::Require(parts&&parts->topology(),"Rigid scene forecast lacks genuine PART topology");
        s.rigid_groups={parts->topology()->source_instance_id(),rigid.groups().size(),rigid.members().size(),
            parts->roots().size(),rigid.plain_source_instance_id()};
    }
    return s;
}
f::NodalCinStartup Cin(const PhysicalSource& p,const DynamicsConfig& c,const double* m,const double* j) {
    return {&p.cin(),m,j,nullptr,nullptr,0,c.qualification};
}
f::NodalCinWitnessSource Witnesses(const PhysicalSource& p){return {&p.cin(),nullptr,nullptr,0,0};}
f::qeph::QephBatchConfig QuadConfig(const PhysicalSource& p,const DynamicsConfig& c,const f::NodalStamp& s) {
    f::qeph::QephBatchConfig q;q.owner=s;q.configuration_id=c.configuration;q.qualification_id=c.qualification;
    q.element_count=p.physical().shells()->qeph_count();q.usage=f::qeph::BatchUsage::CoupledForces;q.startup=p.startup();
    q.storage_limits={1024,2048,32u<<20};q.max_device_bytes=8u<<20;return q;
}
f::t3::T3BatchConfig TriangleConfig(const PhysicalSource& p,const DynamicsConfig& c,const f::NodalStamp& s) {
    f::t3::T3BatchConfig t;t.owner=s;t.configuration_id=c.configuration;t.qualification_id=c.qualification;
    t.element_count=p.physical().shells()->t3_count();t.usage=f::t3::BatchUsage::CoupledForces;t.startup=p.startup();
    t.storage_limits={1024,2048,32u<<20};t.max_device_bytes=8u<<20;return t;
}
f::ShellPublicationLimits PublicationLimits(std::size_t nodes){f::ShellPublicationLimits p;p.max_nodes=nodes;return p;}
}
