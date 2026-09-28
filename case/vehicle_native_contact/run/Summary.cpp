#include "State.h"
#include "../activity/Declaration.h"
#include "case/vehicle_runtime/SolidReadback.h"
#include "case/vehicle_run/SampledShellPlasticity.h"
#include "case/vehicle_run/MechanicsDocument.h"
#include "case/vehicle_run/StageTimingDocument.h"
#include "output/physical_run/Metadata.h"
#include "output/BoundedArrayJson.h"
#include <cmath>
namespace crash::cases::vehicle_native_contact::run_detail {
records::RecordFile WriteSummary(const std::filesystem::path& root, const VehicleContactStartup& source,
    const RunConfig& config, const RunForecast& forecast, const vehicle_run::Horizon& horizon, const RunResult& result) {
    using namespace output;
    Document doc;
    doc.SetObject();
    String(doc, "schema", "robo_dyna.native_vehicle_contact_run.v1");
    const bool controlled=vehicle_runtime::detail::HasSourceSolidControls(source.owner_source().execution_source().mechanical().solids());
    String(doc, "physical_profile", controlled?"native_v6_raw8_heph_explicit_cin28_with_declared_finite_mesh_wall":
        "selected_vehicle_supports_v5_with_declared_finite_mesh_wall");
    String(doc, "contact_profile", "source_type25_self_and_all_retained_nodes_to_fixed_mesh");
    if (source.config().activity == n::ContactActivityPolicy::ShellRemoval)
        String(doc, "contact_activity_profile", "source_declared_shell_removal_fixed_solid_topology_positive_cin");
    String(doc, "initial_state", "source_produced_starter_history_and_final_type2_removals");
    if (source.activity_source())
        array_json::Child(doc, "contact_activity_source", activity::Document(*source.activity_source()));
    Boolean(doc, "visualization_only_not_restart", true);
    Boolean(doc, "session_initialized", result.session_initialized);
    Boolean(doc, "valid_closed_archive", result.loop.valid_manifest && result.archive_manifest.has_value());
    Boolean(doc, "horizon_complete", result.loop.kind == vehicle_run::StopKind::Completed);
    Integer(doc, "stop_kind", static_cast<unsigned>(result.loop.kind));
    String(doc, "stop_reason", result.loop.reason);
    Integer(doc, "accepted_intervals", result.loop.progress.accepted.epoch);
    Number(doc, "actual_time_s", result.loop.progress.accepted.time_s);
    Integer(doc, "planned_intervals", horizon.intervals);
    Number(doc, "fixed_dt_s", horizon.fixed_dt_s);
    Number(doc, "requested_duration_s", horizon.requested_duration_s);
    Number(doc, "session_startup_s", result.session_startup_s);
    String(doc, "startup_timing_scope", "physical owner, both genuine contact initializations, capture and static archive; caller records source construction separately");
    Number(doc, "initial_retry_probe_s", result.retry_probe_s);
    Boolean(doc, "initial_retry_verified", result.initial_retry_verified);
    Number(doc, "runtime_elapsed_s", result.loop.progress.elapsed_s);
    Number(doc, "runtime_steps_per_s", result.loop.progress.accepted_intervals_per_second);
    Number(doc, "step_s", result.loop.progress.timing.step_s);
    Number(doc, "commit_s", result.loop.progress.timing.commit_s);
    Number(doc, "archive_s", result.loop.progress.timing.archive_s);
    Number(doc, "capture_s", result.loop.progress.timing.capture_s);
    Boolean(doc, "stage_profiling_enabled", source.config().dynamics.timing.enabled);
    array_json::Child(doc, "mechanics_stage_timing",
        vehicle_run::detail::StageTimingDocument(result.loop.progress.mechanics_timing));
    Integer(doc, "complete_peak_host_bytes", forecast.complete_peak_host_bytes);
    Integer(doc, "steady_device_bytes", source.forecast().steady_device_bytes);
    Integer(doc, "peak_device_bytes", source.forecast().peak_device_bytes);
    Integer(doc, "archive_forecast_bytes", forecast.archive.archive.archive.forecast_bytes);
    Integer(doc, "archive_cap_bytes", config.archive_bytes);
    String(doc, "motion_metric_scope", "complete physical domain including fixed wall; uniform-translation departure is not vehicle deformation");
    array_json::Child(doc, "mechanics", vehicle_run::detail::MechanicsDocument(result.loop.progress.mechanics));
    array_json::Child(doc, "sampled_shell_plasticity",
        vehicle_run::detail::SampledShellPlasticityDocument(result.loop.progress.sampled_shell_plasticity));
    Value interfaces(rapidjson::kArrayType);
    for (std::size_t i = 0; i < result.native.size(); ++i) {
        Document one;
        one.SetObject();
        const auto& totals = result.native[i];
        const auto& ordered = source.interface_order()[i];
        String(one, "role", ordered.role == Role::Self ? "self" : "mesh_wall");
        Integer(one, "native_id", ordered.native_id);
        Integer(one, "native_storage_ordinal", ordered.native_storage_ordinal);
        Integer(one, "accepted_intervals", totals.intervals);
        Integer(one, "active_force_intervals", totals.active_force_intervals);
        Integer(one, "peak_active_forces", totals.peak_active_forces);
        Integer(one, "peak_raw_candidates", totals.peak_raw_candidates);
        Integer(one, "peak_optimized_candidates", totals.peak_optimized_candidates);
        Integer(one, "first_active_epoch", totals.first_active_epoch);
        Number(one, "first_active_time_s", totals.first_active_time_s);
        if (source.config().activity == n::ContactActivityPolicy::ShellRemoval) {
            Integer(one, "activity_changes", totals.activity_changes);
            Integer(one, "removed_contact_mains", totals.removed_mains);
            Integer(one, "removed_orphan_secondaries", totals.orphan_secondaries);
            Integer(one, "observed_removed_qeph", totals.observed_removed_qeph);
            Integer(one, "observed_removed_t3", totals.observed_removed_t3);
            Integer(one, "first_removal_epoch", totals.first_removal_epoch);
            Number(one, "first_removal_time_s", totals.first_removal_time_s);
            Integer(one, "first_removed_qeph_id", totals.first_removed_qeph_id);
            Integer(one, "first_removed_t3_id", totals.first_removed_t3_id);
        }
        const auto& initial = result.initialization[i];
        Boolean(one, "initialization_available", initial.available);
        if (initial.available) {
            Integer(one, "initial_pairs", initial.values.pairs);
            Integer(one, "initial_warm_before_tied", initial.values.warm_before_tied);
            Integer(one, "initial_warm_after_tied", initial.values.warm_after_tied);
            Integer(one, "initial_tied_reset", initial.values.tied_reset);
            Integer(one, "initial_warm_negative_main", initial.values.warm_negative_main);
        }
        interfaces.PushBack(Value(one, doc.GetAllocator()), doc.GetAllocator());
    }
    doc.AddMember("interfaces", interfaces, doc.GetAllocator());
    if (result.qeph_rejection)
        array_json::Child(doc, "qeph_rejection_capture",
            vehicle_dynamics::diagnostics::qeph_rejection::ExportDocument(*result.qeph_rejection));
    if (result.rejected_native) {
        Document failure;
        failure.SetObject();
        Integer(failure, "operation", static_cast<unsigned>(result.rejected_native->operation));
        Integer(failure, "status", static_cast<unsigned>(result.rejected_native->status));
        Integer(failure, "row", result.rejected_native->row);
        Integer(failure, "occurrence", result.rejected_native->occurrence);
        Integer(failure, "selection_status", static_cast<unsigned>(result.rejected_native->selection_status));
        const auto& rejected = *result.rejected_native;
        Boolean(failure, "source_available", rejected.source.available);
        if (rejected.source.available) Integer(failure, "source_id", rejected.source.source_id);
        Boolean(failure, "candidate_rebuild_available", rejected.diagnostics.candidate_rebuild_available);
        if (rejected.diagnostics.candidate_rebuild_available) {
            const auto& report = rejected.diagnostics.candidate_rebuild;
            Document rebuild;
            rebuild.SetObject();
            String(rebuild, "scope", "last attempted inventory stage; raw report counters, not a complete pair census claim");
            Integer(rebuild, "status", static_cast<unsigned>(report.status));
            Integer(rebuild, "enumeration_strategy", static_cast<unsigned>(report.strategy));
            Boolean(rebuild, "encounters_counted", report.encounters_counted);
            Boolean(rebuild, "pairs_counted", report.pairs_counted);
            Integer(rebuild, "failure_row", report.failure_row);
            Integer(rebuild, "source_id", report.stamp.source.source);
            Integer(rebuild, "topology_generation", report.stamp.source.topology);
            Integer(rebuild, "activity_generation", report.stamp.activity);
            Integer(rebuild, "gap_generation", report.stamp.gaps);
            Integer(rebuild, "geometry_generation", report.stamp.geometry);
            Integer(rebuild, "attempt", report.stamp.attempt);
            Integer(rebuild, "reference_generation", report.stamp.reference);
            Integer(rebuild, "reported_active_secondaries", report.active_secondaries);
            Integer(rebuild, "reported_envelope_encounters", report.envelope_encounters);
            Integer(rebuild, "reported_tasks", report.tasks);
            Integer(rebuild, "reported_pairs", report.pairs);
            Integer(rebuild, "own_kernel_launches", report.own_kernel_launches);
            Integer(rebuild, "sort_calls", report.sort_calls);
            Integer(rebuild, "scan_calls", report.scan_calls);
            Integer(rebuild, "host_fences", report.host_fences);
            Boolean(rebuild, "maximum_secondary_gap_finite", std::isfinite(report.maximum_secondary_gap));
            if (std::isfinite(report.maximum_secondary_gap)) Number(rebuild, "maximum_secondary_gap", report.maximum_secondary_gap);
            for (const auto& entry : source.interface_order()) if (entry.native_id == report.stamp.source.source) {
                const auto index = entry.role == Role::Self ? 0u : 1u;
                Integer(rebuild, "encounter_capacity", source.config().transaction[index].inventory.max_encounters);
                Integer(rebuild, "task_capacity", source.config().transaction[index].inventory.max_tasks);
                Integer(rebuild, "pair_capacity", source.config().transaction[index].inventory.max_pairs);
            }
            array_json::Child(failure, "candidate_rebuild", rebuild);
        }
        array_json::Child(doc, "rejected_native", std::move(failure));
    }
    if (result.rejected_step_limit_s) Number(doc, "rejected_step_limit_s", *result.rejected_step_limit_s);
    if (result.archive_manifest) array_json::Child(doc, "archive_manifest", physical_run::FileDocument(*result.archive_manifest));
    if (result.viewer_input) array_json::Child(doc, "viewer_input", physical_run::FileDocument(*result.viewer_input));
    if (!result.viewer_error.empty()) String(doc, "viewer_error", result.viewer_error);
    return physical_run::WriteDocument(root, "summary.json", doc, physical_run::MetadataCap);
}
} // namespace crash::cases::vehicle_native_contact::run_detail
