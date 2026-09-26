#include "ParticipantConfigs.h"
#include "ParticipantControls.h"
namespace crash::cases::vehicle_runtime::detail {
namespace fe = tl::fea;
namespace {
fe::NodalStamp CapacityStamp(const Config& config,std::size_t nodes,
    const fe::NodalRigidAssemblyBinding& rigid) noexcept {
    fe::NodalStamp stamp;
    // Prospective capacity only; every actual participant uses owner.accepted().
    stamp.owner_id=1;stamp.node_count=nodes;stamp.fixed_dt=config.reserved_step_s;
    stamp.has_rotations=true;stamp.has_rotation_presence=true;
    stamp.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    stamp.rigid_groups={rigid.parts()->topology()->source_instance_id(),rigid.groups().size(),
        rigid.members().size(),rigid.parts()->roots().size(),rigid.plain_source_instance_id()};
    return stamp;
}
}
ParticipantConfigs ConfigureParticipants(const Config& config,const Source& source,const fe::NodalStamp& stamp) {
    ParticipantConfigs out;
    SetParticipantIdentity(out.qeph,config,stamp,source.startup());
    SetParticipantIdentity(out.t3,config,stamp,source.startup());
    SetParticipantIdentity(out.qbat,config,stamp,source.startup());
    out.type25 = fe::type25::BatchConfig::Vehicle();
    SetParticipantIdentity(out.type25,config,stamp,source.startup());
    SetParticipantIdentity(out.type13,config,stamp,source.startup());
    SetParticipantIdentity(out.solids,config,stamp,source.startup());
    out.qeph.element_count = source.physical().shells()->qeph_count();
    out.t3.element_count = source.physical().shells()->t3_count();
    out.qbat.element_count = source.physical().shells()->qbat_count();
    out.qeph.usage = fe::qeph::BatchUsage::CoupledForces;
    out.t3.usage = fe::t3::BatchUsage::CoupledForces;
    out.qbat.usage = fe::qbat::BatchUsage::CoupledForces;
    out.qeph.storage_limits = out.t3.storage_limits = out.qbat.storage_limits = config.limits.shells;
    out.qeph.max_device_bytes = out.t3.max_device_bytes = out.qbat.max_device_bytes = config.limits.shell_device_bytes;
    out.type25.element_count = source.coefficients().type25()->connection_count();
    out.type13.assembly = fe::type13::BatchAssembly::CinNativeStiffness;
    out.solids.profile = source.solids().profile() == fe::solids::ModelProfile::ExtendedLaw44Law90
        ? fe::solids::BatchProfile::PhysicalCinExtendedLaw44Law90V2
        : fe::solids::BatchProfile::PhysicalCinV1;
    out.solids.cin_attachment_count = source.witnesses().data().ranges.size();
    out.solids.cin_witness_count = source.witnesses().data().witnesses.size();
    out.publication = {config.configuration_id,config.qualification_id,source.startup()};
    return out;
}
fe::NodalStamp DescriptiveStamp(const Config& config,const Source& source) noexcept {
    return CapacityStamp(config,source.physical().domain()->node_count(),source.rigid());
}
fe::NodalCinStartup CinStartup(const Config& config,const Source& source,
    const double* mass,const double* inertia) noexcept {
    return MakeCinStartup(config,source.witness_source(),mass,inertia);
}
ParticipantConfigs ConfigureParticipants(const Config& config,const Execution& execution,
    const Attachments& attachments,const fe::NodalStamp& stamp) {
    return ConfigureParticipants(config,Source::Original(execution,attachments),stamp);
}
fe::NodalStamp DescriptiveStamp(const Config& config,const Execution& source) noexcept {
    return CapacityStamp(config,source.physical().domain()->node_count(),source.model().rigid_assembly());
}
fe::NodalCinStartup CinStartup(const Config& config,const Attachments& source,
    const double* mass,const double* inertia) noexcept {
    return MakeCinStartup(config,Witnesses(source),mass,inertia);
}
} // namespace crash::cases::vehicle_runtime::detail
