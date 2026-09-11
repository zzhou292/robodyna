#include "ParticipantConfigs.h"
namespace crash::cases::vehicle_runtime::detail {
namespace fe = tl::fea;
namespace {
template<class C> void Common(C& out,const Config& config,const fe::NodalStamp& stamp) {
    out.owner = stamp;
    out.configuration_id = config.configuration_id;
    out.qualification_id = config.qualification_id;
    out.startup = InitialTranslation();
}
}
ParticipantConfigs ConfigureParticipants(const Config& config,const Execution& source,
    const Attachments& attachments,const fe::NodalStamp& stamp) {
    ParticipantConfigs out;
    Common(out.qeph,config,stamp);
    Common(out.t3,config,stamp);
    Common(out.qbat,config,stamp);
    out.type25 = fe::type25::BatchConfig::Vehicle();
    Common(out.type25,config,stamp);
    Common(out.type13,config,stamp);
    Common(out.solids,config,stamp);
    out.qeph.element_count = source.physical().shells()->qeph_count();
    out.t3.element_count = source.physical().shells()->t3_count();
    out.qbat.element_count = source.physical().shells()->qbat_count();
    out.qeph.usage = fe::qeph::BatchUsage::CoupledForces;
    out.t3.usage = fe::t3::BatchUsage::CoupledForces;
    out.qbat.usage = fe::qbat::BatchUsage::CoupledForces;
    out.qeph.storage_limits = out.t3.storage_limits = out.qbat.storage_limits = config.limits.shells;
    out.qeph.max_device_bytes = out.t3.max_device_bytes = out.qbat.max_device_bytes = config.limits.shell_device_bytes;
    out.type25.element_count = source.model().coefficients().type25()->connection_count();
    out.type13.assembly = fe::type13::BatchAssembly::CinNativeStiffness;
    out.solids.profile = fe::solids::BatchProfile::PhysicalCinV1;
    out.solids.cin_attachment_count = attachments.witnesses().data().ranges.size();
    out.solids.cin_witness_count = attachments.witnesses().data().witnesses.size();
    out.publication = {config.configuration_id,config.qualification_id,InitialTranslation()};
    return out;
}
fe::NodalStamp DescriptiveStamp(const Config& config,const Execution& source) noexcept {
    fe::NodalStamp stamp;
    // Prospective capacity descriptor only; this ID is never used for a runtime
    // operation. All actual participant configs use owner.accepted() instead.
    stamp.owner_id = 1;
    stamp.node_count = source.physical().domain()->node_count();
    stamp.fixed_dt = config.reserved_step_s;
    stamp.has_rotations = true;
    stamp.has_rotation_presence = true;
    stamp.temporal_scheme = fe::NodalTemporalScheme::StaggeredHalfKickStart;
    const auto& rigid = source.model().rigid_assembly();
    stamp.rigid_groups = {rigid.parts()->topology()->source_instance_id(),rigid.groups().size(),
        rigid.members().size(),rigid.parts()->roots().size(),rigid.plain_source_instance_id()};
    return stamp;
}
fe::NodalCinStartup CinStartup(const Config& config,const Attachments& source,
    const double* mass,const double* inertia) noexcept {
    const auto& data = source.witnesses().data();
    return {&source.attachments().model(),mass,inertia,data.ranges.data(),data.witnesses.data(),
            data.witnesses.size(),config.qualification_id};
}
} // namespace crash::cases::vehicle_runtime::detail
