#include "Forecast.h"
#include "ParticipantConfigs.h"
#include "Packing.h"
#include "JointRuntime.h"
#include "lib_src/elements/ShellBatchLayeredSection.h"
#include "lib_utils/BoundedArena.h"
#include "output/ArtifactIO.h"
#include <algorithm>
namespace crash::cases::vehicle_runtime::detail {
namespace fe = tl::fea;
Forecast ForecastStartup(const Config& config,const Execution& execution,const Attachments& attachments,
    std::size_t fixed_bytes,const JointModel* joints) {
    using output::Require;
    CheckConfig(config);
    CheckSource(execution,attachments);
    const auto& physical = execution.physical();
    const auto cin = Witnesses(attachments);
    const auto c = ConfigureParticipants(config,execution,attachments,DescriptiveStamp(config,execution));
    Forecast out;
    out.prior_construction_bytes = {execution.model().forecast().total_bytes,execution.forecast().total_bytes,
                                   attachments.forecast().total_bytes};
    out.app_fixed_bytes = fixed_bytes;
    if(joints) ForecastJoints(config,execution,attachments,*joints,out);
    out.retained_source_upper_bound = SourceBytes(execution,attachments,config.limits.host_bytes);
    const auto nodes = physical.domain()->node_count();
    out.packing_bytes = PackingBytes(nodes,config.limits.host_bytes);
    out.owner = fe::FENodalState::ForecastAssemblyCin(OwnerConfig(config,nodes),
        execution.model().rigid_assembly(),CinStartup(config,attachments),true);
    Require(out.owner.report.status == fe::NodalStatus::Ok,out.owner.report.message);
    const auto q = fe::qeph::QephBatch::ForecastMapped(c.qeph,physical,cin,config.limits.failure);
    const auto t = fe::t3::T3Batch::ForecastMapped(c.t3,physical,cin,config.limits.failure);
    const auto b = fe::qbat::Batch::ForecastMapped(c.qbat,physical,cin);
    const auto s = fe::type25::Batch::ForecastMapped(c.type25,physical,cin,fe::type25::CapacityProfile::Vehicle);
    Require(q.report.status == fe::qeph::BatchStatus::Success,q.report.message);
    Require(t.report.status == fe::t3::BatchStatus::Success,t.report.message);
    Require(b.report.status == fe::qbat::BatchStatus::Success,b.report.message);
    Require(s.report.status == fe::type25::BatchStatus::Success,s.report.message);
    out.participants[0] = q.footprint;
    out.participants[1] = t.footprint;
    out.participants[2] = b.footprint;
    out.participants[3] = s.footprint;
    fe::type13::BatchForecast beam;
    const auto beam_report = fe::type13::Batch::ForecastMapped(c.type13,physical,
        execution.model().rigid_assembly(),cin,beam,config.limits.beams);
    Require(bool(beam_report),beam_report.message);
    fe::solids::BatchForecast solid;
    const auto solid_report = fe::solids::Batch::Forecast(c.solids,execution.model().solids(),solid);
    Require(bool(solid_report),solid_report.message);
    // These two existing APIs expose a complete bound. Charge all incremental
    // scratch as retained, conservatively, rather than duplicate private math.
    Require(beam.startup_host_bytes >= physical.owned_payload_bytes() &&
        solid.startup_host_bytes >= execution.model().solids().owned_payload_bytes(),
        "Complete participant source partition is inconsistent");
    out.participants[4] = {beam.device_bytes,physical.owned_payload_bytes(),
        beam.startup_host_bytes-physical.owned_payload_bytes(),0,beam.startup_host_bytes};
    out.participants[5] = {solid.device_bytes,execution.model().solids().owned_payload_bytes(),
        solid.startup_host_bytes-execution.model().solids().owned_payload_bytes(),0,solid.startup_host_bytes};
    const auto publication = fe::ShellBatchPublication::ForecastPhysical(physical,cin.range_count,
        config.limits.publisher,out.publisher);
    Require(publication.status == fe::ShellPublicationStatus::Success,publication.message);
    tl::util::BoundedArenaLayout host(config.limits.host_bytes),device(config.limits.device_bytes);
    tl::util::ArenaRegion unused;
    for (auto bytes : {out.retained_source_upper_bound,fixed_bytes,nodes,out.owner.owner_host_bytes,
                      out.publisher.owned_host_bytes})
        Require(host.Append<std::byte>(bytes,unused),"Retained runtime host payload exceeds cap");
    Require(device.Append<std::byte>(out.owner.device_bytes,unused) &&
        device.Append<std::byte>(out.publisher.device_bytes,unused),"Owner/publication device bytes exceed cap");
    out.peak_temporary_bytes = out.packing_bytes + out.owner.startup_scratch_bytes;
    for (const auto& participant : out.participants) {
        Require(host.Append<std::byte>(participant.participant_host_bytes,unused) &&
            device.Append<std::byte>(participant.device_bytes,unused),"Complete participant allocation exceeds runtime cap");
        out.peak_temporary_bytes = std::max(out.peak_temporary_bytes,participant.startup_scratch_bytes);
    }
    if(joints) {
        // The wrapper discounts only its exact shared physical/rigid backing.
        // Batch scratch is conservatively retained for the entire run.
        Require(host.Append<std::byte>(out.joint_source_bytes,unused) &&
            host.Append<std::byte>(out.joints.incremental_host_bytes,unused) &&
            device.Append<std::byte>(out.joints.device_bytes,unused),
            "Complete joint runtime exceeds the physical host/device cap");
    }
    out.peak_temporary_bytes = std::max(out.peak_temporary_bytes,
        out.publisher.startup_host_bytes-out.publisher.owned_host_bytes);
    // Native readbacks require complete family shapes. Visit channels/families
    // sequentially and retire each buffer before the next one is allocated.
    const std::size_t reads[]{out.packing_bytes,
        c.qeph.element_count*sizeof(fe::qeph::ForceTrial),c.t3.element_count*sizeof(fe::t3::ForceTrial),
        c.qbat.element_count*sizeof(fe::qbat::BatchResult),
        c.qeph.element_count*sizeof(fe::ShellBatchLayeredSection),c.t3.element_count*sizeof(fe::ShellBatchLayeredSection),
        c.type25.element_count*sizeof(fe::type25::Evaluation),
        execution.model().beams().connection_count()*sizeof(fe::type13::Evaluation),
        execution.model().solids().solid18().size()*sizeof(fe::solids::Result18) +
        execution.model().solids().solid24().size()*sizeof(fe::solids::Result24) +
        execution.model().solids().solid6z().size()*sizeof(fe::solids::Result6z)};
    out.readback_temporary_bytes = *std::max_element(std::begin(reads),std::end(reads));
    if(joints) out.readback_temporary_bytes=std::max(out.readback_temporary_bytes,
        joints->model().joints().size()*sizeof(fe::type45::Result));
    out.peak_temporary_bytes = std::max(out.peak_temporary_bytes,out.readback_temporary_bytes);
    out.retained_host_upper_bound = host.bytes();
    Require(host.Append<std::byte>(out.peak_temporary_bytes,unused),"Peak runtime host phase exceeds cap");
    out.peak_host_upper_bound = host.bytes();
    out.device_bytes = device.bytes();
    return out;
}
} // namespace crash::cases::vehicle_runtime::detail
