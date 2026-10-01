#include "Session.h"

namespace robodyna::driver {
crash::output::Document PlanDocument(const Request& request, const Prepared& prepared, const char* mode) {
    namespace io = crash::output;
    io::Document report;
    report.SetObject();
    io::String(report, "schema", "robodyna.native_vehicle_driver.v1");
    io::String(report, "mode", mode);
    io::String(report, "profile", "yaris.native_v6.wall_self");
    io::Boolean(report, "physical_owner_created", false);
    io::Boolean(report, "source_preparation_used_cuda", true);
    io::Boolean(report, "physics_prepared", true);
    io::Boolean(report, "horizon_complete", false);
    io::Boolean(report, "full_cpp_replay_verified", false);
    io::Boolean(report, "physical_restart", false);
    io::Number(report, "requested_duration_s", request.run.duration_s);
    io::Number(report, "fixed_dt_s", request.run.fixed_dt_s);
    io::Integer(report, "samples", request.run.samples);
    const auto& forecast = prepared.run.forecast();
    io::Integer(report, "planned_intervals", forecast.archive.archive.archive.frame_epochs.back());
    io::Integer(report, "source_construction_peak_bytes", prepared.originals.forecast().peak_bytes);
    io::Integer(report, "source_extra_retained_bytes", prepared.source_extra_bytes);
    io::Integer(report, "complete_peak_host_bytes", forecast.complete_peak_host_bytes + prepared.source_extra_bytes + ExportBytes);
    io::Integer(report, "steady_device_bytes", prepared.source.forecast().steady_device_bytes);
    io::Integer(report, "peak_device_bytes", prepared.source.forecast().peak_device_bytes);
    io::Integer(report, "sequential_replay_peak_host_bytes", prepared.replay_peak_bytes);
    io::Integer(report, "archive_forecast_bytes", forecast.archive.archive.archive.forecast_bytes);
    io::Integer(report, "archive_cap_bytes", request.resources.archive_bytes);
    io::Integer(report, "host_guard_bytes", request.resources.rss_bytes);
    io::Boolean(report, "fits_runtime_limits", forecast.fits_runtime_limits);
    return report;
}
}  // namespace robodyna::driver
