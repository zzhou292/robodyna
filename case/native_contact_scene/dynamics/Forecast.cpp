#include "Internal.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>
namespace crash::cases::native_scene {
namespace dd=dynamics_detail;
DynamicsForecast NativeSceneDynamics::Preflight(const ContactSource& source,DynamicsConfig config) {
    namespace f=tl::fea;using output::Require;const auto& p=source.physical_source();const auto& physical=p.physical();
    const auto owner_config=dd::OwnerConfig(p,config);const auto nodes=physical.domain()->node_count();
    const auto stamp=dd::DescriptiveStamp(p,config);const auto witnesses=dd::Witnesses(p);
    DynamicsForecast out;out.source_host_bytes=p.retained_host_upper_bound()+source.forecast().owned_bytes;
    out.packing_bytes=vehicle_runtime::detail::PackingBytes(nodes,config.limits.host_bytes)+nodes;
    out.owner=f::FENodalState::ForecastAssemblyCin(owner_config,p.rigid(),dd::Cin(p,config),true);
    dd::RequireSuccess(out.owner.report);
    const auto q=f::qeph::QephBatch::ForecastMapped(dd::QuadConfig(p,config,stamp),physical,witnesses,{});
    const auto t=f::t3::T3Batch::ForecastMapped(dd::TriangleConfig(p,config,stamp),physical,witnesses,{});
    dd::RequireSuccess(q.report);dd::RequireSuccess(t.report);out.qeph=q.footprint;out.t3=t.footprint;
    dd::RequireSuccess(f::ShellBatchPublication::ForecastPhysical(physical,0,dd::PublicationLimits(nodes),out.publication));
    // A prospective host-only issuer is used for the public byte forecast. It
    // is never bound, registered or used as an accepted/physical authority.
    f::ShellPhysicalScratchParticipation prospective;
    dd::RequireSuccess(f::ShellBatchPublication::ForecastPhysicalScratchParticipation(
        {{},{&prospective,source.source().source_id}},{},out.roster));
    out.transaction_host_reservation=config.limits.contact.max_host_bytes;
    out.transaction_device_reservation=config.limits.contact.max_device_bytes;
    Require(out.transaction_host_reservation&&out.transaction_device_reservation,
        "Native transaction reservation must be explicit and positive");
    tl::util::BoundedArenaLayout host(config.limits.host_bytes),device(config.limits.device_bytes);tl::util::ArenaRegion unused;
    for(auto bytes:{sizeof(Storage),out.source_host_bytes,out.owner.owner_host_bytes,out.qeph.participant_host_bytes,
        out.t3.participant_host_bytes,out.publication.owned_host_bytes,out.roster.publication_host_bytes})
        Require(host.Append<std::byte>(bytes,unused),"Native dynamics retained host storage exceeds cap");
    // Transaction maximum host bytes already includes its complete startup
    // source staging. Reserve it once and keep this conservative bound live.
    Require(host.Append<std::byte>(out.transaction_host_reservation,unused),"Native transaction host reservation exceeds cap");
    out.retained_host_bytes=host.bytes();
    const auto temporary=std::max({out.packing_bytes+out.owner.startup_scratch_bytes,out.qeph.startup_scratch_bytes,
        out.t3.startup_scratch_bytes,out.publication.startup_host_bytes-out.publication.owned_host_bytes});
    Require(host.Append<std::byte>(temporary,unused),"Native dynamics startup peak exceeds cap");out.peak_host_bytes=host.bytes();
    for(auto bytes:{out.owner.device_bytes,out.qeph.device_bytes,out.t3.device_bytes,out.publication.device_bytes,out.transaction_device_reservation})
        Require(device.Append<std::byte>(bytes,unused),"Native dynamics complete device reservation exceeds cap");
    out.device_bytes=device.bytes();return out;
}
}
