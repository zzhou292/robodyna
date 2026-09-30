#include "RunInternal.h"
#include "output/BoundedArrayJson.h"
namespace crash::cases::native_scene::run_detail {
namespace {
const char* Stop(vehicle_run::StopKind value) {
    using K=vehicle_run::StopKind;
    switch(value){case K::Completed:return "completed";case K::Requested:return "requested_stop";
    case K::IntervalLimit:return "diagnostic_interval_limit";case K::TimeLimit:return "elapsed_limit";
    case K::StartupFailure:return "startup_failure";case K::PhysicsRejected:return "physics_rejected";
    case K::ArchiveFailure:return "archive_failure";case K::CaptureFailure:return "capture_failure";
    case K::ObserverFailure:return "observer_failure";}return "unknown";
}
}
output::Document ForecastDocument(const RunConfig& config,const vehicle_run::Horizon& h,const RunForecast& f,const ContactSelection& source) {
    using namespace output;Document d;d.SetObject();String(d,"schema","robo_dyna.native_shell_impact_forecast.v1");
    String(d,"case","declared_gpu_shell_impact_coupon");String(d,"scope","capacity_bounds_not_stability_or_completed_physics");
    Integer(d,"run_id",config.run_id);Integer(d,"planned_intervals",h.intervals);Integer(d,"samples",config.samples);
    Number(d,"fixed_dt_s",h.fixed_dt_s);Number(d,"descriptive_duration_s",h.requested_duration_s);
    Integer(d,"host_upper_bound_bytes",f.host_upper_bound);Integer(d,"device_payload_upper_bound_bytes",f.device_upper_bound);
    Integer(d,"archive_upper_bound_bytes",f.archive.archive.archive.forecast_bytes);
    Integer(d,"transaction_host_cap_reservation_bytes",f.dynamics.transaction_host_reservation);
    Integer(d,"transaction_device_cap_reservation_bytes",f.dynamics.transaction_device_reservation);
    String(d,"memory_scope","module_payload_bounds_exclude_allocator_and_cuda_driver_context_memory");
    const auto& declared=source.physical_source().declared().data();const auto& hardening=source.physical_source().hardening();
    String(d,"declared_export_sha256",declared.export_sha256);String(d,"source_json_sha256",declared.definition_sha256);
    Number(d,"source_h_pa",hardening.source_h_pa);Number(d,"derived_etan_pa",hardening.derived_etan_pa);
    Number(d,"prepared_h_pa",hardening.prepared_h_pa);Integer(d,"prepared_h_ulp_difference",hardening.prepared_h_ulp_difference);
    Boolean(d,"exact_source_h_identity",hardening.exact_source_h_identity);Number(d,"projection_working_length_m",source.config().units.length_m);
    String(d,"native_profile",source.profile_name());
    String(d,"native_preprocessing",source.preprocessing()==native::search_startup::Initialization::SerialNative?"serial_native":"proved_no_expansion");
    Integer(d,"native_workers",source.source().native_workers);Integer(d,"native_force_packet_size",source.source().force_packet_size);
    Integer(d,"nodes",source.source().selection.node_count);Integer(d,"primary_mains",source.source().primary_main_count);
    Integer(d,"expanded_mains",source.source().selection.main_count);Integer(d,"secondaries",source.source().selection.secondary_count);
    return d;
}
output::full_shell::RecordFile Summary(const std::filesystem::path& root,const RunConfig& config,const vehicle_run::Horizon& h,
    const RunForecast& f,const ContactSelection& source,const RunResult& result) {
    using namespace output;Document d;d.SetObject();String(d,"schema","robo_dyna.native_shell_impact_run.v1");
    String(d,"caption","GPU shell-impact coupon");String(d,"scope","accepted_visualization_not_vehicle_delivery_or_restart");
    array_json::Child(d,"forecast",ForecastDocument(config,h,f,source));
    String(d,"stop_kind",Stop(result.loop.kind));String(d,"reason",result.loop.reason);
    Integer(d,"accepted_intervals",result.loop.progress.accepted.epoch);Number(d,"actual_time_s",result.loop.progress.accepted.time_s);
    Boolean(d,"requested_steps_reached",result.loop.progress.accepted.epoch==h.intervals);
    Boolean(d,"valid_closed_archive",result.loop.valid_manifest&&result.manifest.has_value());
    Number(d,"startup_wall_s",result.startup_wall_s);Number(d,"run_wall_s",result.loop.progress.elapsed_s);
    Number(d,"prepare_wall_s",result.loop.progress.timing.step_s);Number(d,"commit_wall_s",result.loop.progress.timing.commit_s);
    Number(d,"capture_wall_s",result.loop.progress.timing.capture_s);Number(d,"archive_wall_s",result.loop.progress.timing.archive_s);
    String(d,"output_error",result.output_error);
    if(result.manifest)array_json::Child(d,"archive_manifest",physical_run::FileDocument(*result.manifest));
    if(result.viewer_input)array_json::Child(d,"viewer_input",physical_run::FileDocument(*result.viewer_input));
    return physical_run::WriteDocument(root,"summary.json",d,physical_run::MetadataCap);
}
}
