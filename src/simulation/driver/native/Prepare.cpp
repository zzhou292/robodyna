#include "Session.h"
#include "case/vehicle_native_contact/actual/PreviewResources.h"
#include "case/vehicle_native_contact/actual/SolidExecutionLimits.h"
#include "case/vehicle_native_contact/actual/ContactActivitySelection.h"
#include <utility>

namespace robodyna::driver {
Prepared Prepare(const Request& request) {
    namespace io = crash::output;
    // Reuse the exact qualified resource profile. These pure compatibility
    // helpers contain no GoogleTest dependency or environment reads.
    auto config = vehicle::test::PreviewResources();
    config.activity = vehicle::test::ParseContactActivity(request.run.contact_activity.c_str());
    config.dynamics.startup.reserved_step_s = request.run.fixed_dt_s;
    const auto worker_text = std::to_string(request.resources.solid_worker_blocks);
    config.dynamics.startup.limits.solids = vehicle::test::SolidExecutionLimits(worker_text.c_str());
    config.dynamics.timing.enabled = request.run.stage_timing;
    config.dynamics.capture_qeph_rejection = request.run.capture_qeph_rejection;
    config.requested_duration_s = request.run.duration_s;

    const auto& paths = request.sources;
    const crash::cases::vehicle_run::OriginalPaths originals{
        paths.at("canonical"), paths.at("scope"), paths.at("member"), paths.at("declarations"),
        paths.at("glass_resolution"), paths.at("type13"), paths.at("auxiliary_member"),
        paths.at("original_wall_member"), paths.at("wall_manifest"), paths.at("self_contact_combine_member")};
    const crash::modelio::solid_control_packets::Artifact packets{
        request.packets.path, request.packets.bytes, request.packets.sha256, "native_v6_raw8_heph_explicit_cin28"};
    auto backing = vehicle::source::OriginalSources::Prepare(
        originals, packets, {request.resources.rss_bytes - ExportBytes});
    const auto input = backing.inputs();
    const auto& source_budget = backing.forecast();
    io::Require(source_budget.retained_bytes >= source_budget.sources.retained_bytes,
                "Invalid retained native source accounting");
    const auto extra = source_budget.retained_bytes - source_budget.sources.retained_bytes;
    io::Require(extra <= request.resources.rss_bytes - ExportBytes, "Original source extras exceed host envelope");
    const auto execution_cap = request.resources.rss_bytes - ExportBytes - extra;
    const auto early = vehicle::VehicleContactStartup::ForecastPreparation(
        input.owner, input.self, input.wall, input.controls, config);
    io::Require(early.host_preparation_ceiling <= execution_cap, "Native preparation exceeds host envelope");
    auto source = vehicle::VehicleContactStartup::Prepare(input.owner, input.self, input.wall, input.controls, config);
    vehicle::RunConfig run_config;
    run_config.samples = request.run.samples;
    run_config.identity.run = UINT64_C(0x4e41545636434152);
    run_config.identity.topology = UINT64_C(0x4e4154563657414c);
    run_config.verify_initial_retry = request.run.verify_initial_retry;
    run_config.archive_bytes = request.resources.archive_bytes;
    run_config.artifact_file_bytes = request.resources.artifact_file_bytes;
    auto run = vehicle::PreparedRun::Prepare(source, run_config);
    io::Require(run.forecast().fits_runtime_limits && run.forecast().complete_peak_host_bytes <= execution_cap,
                "Complete native run/output exceeds declared resource limits");
    const auto& base = source.forecast();
    const auto replay_phase = base.sources.retained_bytes + base.packing_retained + base.prepared_source_retained +
        run.mapping().payload_bytes() + run.forecast().controller_bytes + ReplayBytes + ExportBytes;
    io::Require(replay_phase <= request.resources.rss_bytes - extra, "Sequential replay exceeds host envelope");
    return {std::move(backing), std::move(source), std::move(run), extra, replay_phase + extra};
}
}  // namespace robodyna::driver
