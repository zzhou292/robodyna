#include "RunInternal.h"
#include "output/full_shell/FixedStepHorizon.h"
#include "lib_utils/BoundedArena.h"
namespace crash::cases::native_scene {
PreparedNativeSceneRun PreparedNativeSceneRun::Prepare(const ContactSelection& contact,const ArchiveSource& archive,RunConfig config) {
    using output::Require;const auto& physical=contact.physical_source().physical();
    Require(config.run_id&&config.steps&&config.steps<=1000000&&config.samples>=2&&config.samples<=1000&&
        config.samples-1<=config.steps&&config.host_bytes&&config.host_bytes<=1u<<30&&
        config.archive_bytes&&config.archive_bytes<=output::full_shell::TotalByteCap&&
        physical.Matches(archive.physical_source().physical()),"Native run source/count/resource profile differs");
    vehicle_run::Horizon horizon;horizon.intervals=config.steps;horizon.fixed_dt_s=config.dynamics.fixed_dt;
    Require(output::full_shell::PlanExactStepHorizon(horizon.fixed_dt_s,horizon.intervals,horizon.requested_duration_s)&&
        horizon.requested_duration_s<=contact.physical_source().declared().data().end_time_s,
        "Native run exceeds declared source horizon or exact-step domain");
    horizon.nominal_endpoint_s=static_cast<long double>(horizon.fixed_dt_s)*horizon.intervals;
    output::full_shell::Identity id;id.owner=1; // Prospective capacity identity, never a receipt.
    id.run=config.run_id;id.source_instance=physical.domain()->source_instance_id();id.topology=contact.source().topology_generation;
    id.configuration=config.dynamics.configuration;id.qualification=config.dynamics.qualification;
    auto context=archive.mapping().MakeFrameContext(id,horizon.fixed_dt_s);
    auto data=std::make_shared<Data>(contact,archive,config,std::move(context));data->horizon=horizon;
    auto& f=data->forecast;f.dynamics=NativeSceneDynamics::Preflight(contact,config.dynamics);
    f.capture=output::physical_frames::NativeAcceptedFrames::Preflight(archive.mapping(),physical,data->prospective,
        contact.source().selection.secondary_count,config.capture);
    data->request=output::physical_run::MakeRequest(data->prospective,horizon.intervals,horizon.requested_duration_s,
        config.samples,config.archive_bytes);
    f.archive=output::physical_run::RunArchive::Preflight(archive.mapping(),data->prospective,data->request,
        output::physical_frames::NativeAcceptedFrames::ObservationProfile());
    tl::util::BoundedArenaLayout total(config.host_bytes);tl::util::ArenaRegion unused;
    // Existing module bounds include their shared immutable source; keep that
    // conservative inclusion instead of subtracting merely equal byte counts.
    for(auto bytes:{sizeof(Data)+std::size_t(4u<<20),f.dynamics.peak_host_bytes,f.capture.peak_bytes,f.archive.peak_host_bytes})
        Require(total.Append<std::byte>(bytes,unused),"Native run complete source/runtime/output reservation exceeds cap");
    f.host_upper_bound=total.bytes();f.device_upper_bound=f.dynamics.device_bytes;
    return PreparedNativeSceneRun(std::move(data));
}
const RunForecast& PreparedNativeSceneRun::forecast() const noexcept{return data_->forecast;}
const vehicle_run::Horizon& PreparedNativeSceneRun::horizon() const noexcept{return data_->horizon;}
output::Document PreparedNativeSceneRun::ForecastDocument() const {
    return run_detail::ForecastDocument(data_->config,data_->horizon,data_->forecast,data_->contact);
}
}
