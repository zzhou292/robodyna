#include "RunState.h"
#include "source/PhysicalSelection.h"
#include "case/vehicle_runtime/ParticipantConfigs.h"
#include "lib_utils/BoundedArena.h"
#include "output/ArtifactIO.h"
#include <algorithm>
namespace crash::cases::vehicle_run {
namespace {
constexpr std::size_t MappingCap=512u<<20,ControllerReserve=4u<<20;
// Retained/staged sampled values and fixed progress/result copies fit inside
// the existing controller allowance; no per-parent summary array is allocated.
static_assert(8 * sizeof(SampledShellPlasticityTotals) <= 2048 && 2048 < ControllerReserve);
static_assert(sizeof(ContactComposition) < 4096 && sizeof(PreparedRun) < 4096);
std::size_t Sum(std::size_t cap,std::initializer_list<std::size_t> values) {
    tl::util::BoundedArenaLayout budget(cap);
    tl::util::ArenaRegion region;
    for(auto bytes:values) output::Require(budget.Append<std::byte>(bytes,region),"Complete run reservation exceeds cap");
    return budget.bytes();
}
}
PreparedRun PreparedRun::Prepare(const vehicle_wall::VehicleWallSetup& setup,const vehicle_runtime::JointModel& joints,
    Config config,records::Identity identity,
    std::shared_ptr<const vehicle_self_contact::VehicleSelfContactSetup> self_contact) {
    const auto horizon=Plan(config);
    const auto selected=detail::SelectPhysical(config.physical_profile);
    const bool has_beams=setup.execution().model().structural_beams()!=nullptr;
    output::Require(setup.execution().model().source_domain().policy()==selected.domain &&
        joints.source().policy()==selected.joints &&
        joints.model().joints().size()==modelio::type45::detail::Required(selected.joints) &&
        has_beams==selected.structural_beams,
        "Run physical profile differs from the authenticated domain or joint composition");
    output::Require(setup.settings().requested_duration_s==config.duration_s &&
        setup.settings().mesh_profile==vehicle_wall::WallMeshProfile::EnvelopeRectangleV1 &&
        setup.settings().transverse_margin_m>=.25 && identity.run && identity.topology &&
        !identity.owner,
        "Run requires matching loaded envelope/duration and a fresh run identity");
    auto dynamics=vehicle_wall::LoadedWallConfig();
    output::Require((!identity.source_instance || identity.source_instance==setup.execution().physical().domain()->source_instance_id()) &&
        (!identity.configuration || identity.configuration==dynamics.startup.configuration_id) &&
        (!identity.qualification || identity.qualification==dynamics.startup.qualification_id),
        "Run identity conflicts with actual source/configuration/qualification");
    dynamics.startup.reserved_step_s=config.fixed_dt_s;
    dynamics.timing.enabled=true;
    auto contact=ContactComposition::Prepare(config.contact_profile,std::move(self_contact));
    const auto composition=contact.Preflight(setup,dynamics,&joints);
    const auto maximum_host=config.resources==ResourceProfile::Normal?20ull*1000*1000*1000:60ull*1000*1000*1000;
    const auto mapping_phase=Sum(maximum_host,{composition.peak_host_upper_bound,MappingCap,ControllerReserve});
    auto mapping=output::physical_frames::Mapping::Prepare(setup.execution(),MappingCap);
    // Reuse the existing descriptive stamp only for value forecasts. The live
    // capture factory later supplies/authenticates its actual allocated owner.
    auto prospective=identity;
    prospective.owner=vehicle_runtime::detail::DescriptiveStamp(dynamics.startup,setup.execution()).owner_id;
    prospective.source_instance=setup.execution().physical().domain()->source_instance_id();
    prospective.configuration=dynamics.startup.configuration_id;
    prospective.qualification=dynamics.startup.qualification_id;
    const auto context=mapping.source_mapping().MakeFrameContext(prospective,config.fixed_dt_s);
    const auto capture=output::physical_frames::PhysicalAcceptedFrames::Preflight(mapping,context);
    const auto probe_request=output::physical_run::MakeWallRequest(context,horizon.intervals,config.duration_s,
        config.samples,records::FullRunByteCap-CompanionByteCap);
    const output::physical_run::Profile output_profile{
        true,true,has_beams,config.contact_profile==ContactProfile::WallSelfContactV1};
    const auto probe=output::physical_run::RunArchive::PreflightWithWall(setup,mapping,context,probe_request,output_profile);
    const auto shared_wall=Sum(maximum_host,{setup.forecast().shared_source_upper_bound,
        setup.forecast().retained_setup_bytes});
    output::Require(probe.shared_wall_setup_upper_bound==shared_wall &&
        shared_wall<=composition.retained_host_upper_bound,
        "Archive and loaded owner do not share the same retained setup reservation");
    const auto required_host=std::max(mapping_phase,Sum(maximum_host,
        {composition.peak_host_upper_bound,capture.peak_bytes,probe.peak_host_bytes,ControllerReserve}));
    const auto required_archive=Sum(records::FullRunByteCap,{probe.archive.archive.forecast_bytes,CompanionByteCap});
    const auto caps=SelectCaps(config.resources,required_host,required_archive);
    auto next=std::make_shared<Data>(setup,joints,std::move(mapping),config,std::move(identity),std::move(contact));
    next->dynamics=dynamics;
    next->horizon=horizon;
    next->request=output::physical_run::MakeWallRequest(context,horizon.intervals,config.duration_s,
        config.samples,caps.archive_bytes-CompanionByteCap);
    next->forecast.wall=composition.wall;
    next->forecast.contact=composition;
    next->forecast.capture=capture;
    next->forecast.archive=output::physical_run::RunArchive::PreflightWithWall(setup,next->mapping,context,next->request,next->profile);
    next->forecast.complete_host_bytes=required_host;
    next->forecast.complete_archive_bytes=required_archive;
    next->forecast.caps=caps;
    next->forecast.joint_count=joints.model().joints().size();
    return PreparedRun(std::move(next));
}
const Forecast& PreparedRun::forecast() const noexcept {return data_->forecast;}
const Horizon& PreparedRun::horizon() const noexcept {return data_->horizon;}
} // namespace crash::cases::vehicle_run
