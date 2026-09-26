#include "State.h"
#include "output/full_shell/FixedStepHorizon.h"
#include <algorithm>
namespace crash::cases::vehicle_native_contact {
namespace run_detail {
std::size_t Add(std::size_t a, std::size_t b) {
    output::Require(b <= SIZE_MAX - a, "Native accepted-run forecast overflow");
    return a + b;
}
}
PreparedRun PreparedRun::Prepare(const VehicleContactStartup& source, RunConfig config) {
    using run_detail::Add;
    const auto& case_config = source.config();
    const auto& base = source.forecast();
    vehicle_run::Horizon horizon;
    horizon.fixed_dt_s = case_config.dynamics.startup.reserved_step_s;
    horizon.requested_duration_s = case_config.requested_duration_s;
    output::Require(config.samples >= 2 && config.samples <= 1000 && config.mapping_bytes &&
                        config.mapping_bytes <= (512u << 20) && config.archive_bytes &&
                        config.archive_bytes <= records::FullRunByteCap && config.identity.run && config.identity.topology &&
                        !config.identity.owner && !config.identity.source_instance && !config.identity.configuration &&
                        !config.identity.qualification && horizon.requested_duration_s <= .05 &&
                        records::PlanFixedStepHorizon(horizon.fixed_dt_s, horizon.requested_duration_s, horizon.intervals) &&
                        config.samples <= horizon.intervals + 1,
                    "Native run has invalid source horizon/output identity/resource limits");
    horizon.nominal_endpoint_s = static_cast<long double>(horizon.fixed_dt_s) * horizon.intervals;
    const auto cap = case_config.dynamics.startup.limits.host_bytes;
    const auto source_only = Add(Add(base.sources.retained_bytes, base.packing_retained), base.prepared_source_retained);
    // A separate immutable Source wrapper is retained by Mapping. Its actual
    // complete owner backing is the same source handle already charged above.
    const auto view = vehicle_runtime::Source::WithEnvironment(source.owner_source());
    const auto view_bound = view.retained_host_upper_bound(cap);
    output::Require(view_bound >= base.sources.owner_retained, "Output source wrapper retained partition differs");
    const auto wrapper = view_bound - base.sources.owner_retained;
    const auto controller = Add(std::size_t{4} << 20, wrapper);
    const auto preparation = Add(source_only, Add(controller, Add(config.mapping_bytes,
        Add(config.capture.records.host_bytes, Add(config.capture.host_bytes, config.archive.host_bytes)))));
    output::Require(preparation <= cap, "Complete accepted-output preparation exceeds the case host cap");
    auto mapping = output::physical_frames::Mapping::Prepare(view, config.mapping_bytes);
    auto next = std::make_shared<Data>(source, std::move(config), std::move(mapping));
    next->horizon = horizon;
    next->profile.type45 = true;
    next->profile.structural_limit = true;
    next->profile.beam18 = next->mapping.structural_beams() != nullptr;
    next->profile.native_group = true;
    auto id = next->config.identity;
    id.owner = 1; // Count-only record preflight, never a live accepted identity.
    id.source_instance = view.physical().domain()->source_instance_id();
    id.configuration = case_config.dynamics.startup.configuration_id;
    id.qualification = case_config.dynamics.startup.qualification_id;
    const auto context = next->mapping.source_mapping().MakeFrameContext(id, horizon.fixed_dt_s, next->config.capture.records);
    auto& f = next->forecast;
    f.controller_bytes = controller;
    f.retry_bytes = next->config.verify_initial_retry ? std::size_t{64} << 20 : 0;
    f.mapping_bytes = next->mapping.payload_bytes();
    f.preparation_peak_host_bytes = preparation;
    f.capture = output::physical_frames::PhysicalAcceptedFrames::Preflight(next->mapping, context, next->config.capture);
    f.archive = output::physical_run::RunArchive::PreflightWithEnvironment(next->mapping, context,
        output::physical_run::MakeEnvironmentRequest(context, horizon.intervals, horizon.requested_duration_s,
            next->config.samples, next->config.archive_bytes), next->profile, next->config.archive);
    const auto owner_phase = Add(base.peak_host_bytes, Add(f.mapping_bytes, controller));
    // Capture's published bound already contains the same mapping payload.
    // Archive and capture peaks are conservatively allowed to coexist.
    const auto output_phase = Add(base.retained_host_bytes,
        Add(controller, Add(f.capture.peak_bytes, Add(f.archive.peak_host_bytes, f.retry_bytes))));
    f.complete_peak_host_bytes = std::max({preparation, owner_phase, output_phase});
    f.fits_runtime_limits = base.fits_runtime_limits && f.complete_peak_host_bytes <= cap;
    return PreparedRun(std::move(next));
}
const RunForecast& PreparedRun::forecast() const noexcept { return data_->forecast; }
const output::physical_frames::Mapping& PreparedRun::mapping() const noexcept { return data_->mapping; }
} // namespace crash::cases::vehicle_native_contact
