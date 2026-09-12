#include "RunState.h"
#include "MechanicsDocument.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_run::detail {
namespace {
const char* Name(StopKind kind) {
    switch(kind) {
        case StopKind::Completed:return "completed_planned_horizon";
        case StopKind::Requested:return "cooperative_stop";
        case StopKind::IntervalLimit:return "diagnostic_interval_limit";
        case StopKind::TimeLimit:return "elapsed_time_limit";
        case StopKind::StartupFailure:return "startup_session_failure";
        case StopKind::PhysicsRejected:return "physics_rejected";
        case StopKind::ArchiveFailure:return "archive_failure";
        case StopKind::CaptureFailure:return "accepted_capture_failure";
        case StopKind::ObserverFailure:return "observer_failure";
    }
    throw std::invalid_argument("Unknown run stop kind");
}
output::Document Timings(const vehicle_dynamics::StepTimingSnapshot& timing) {
    using namespace output;
    Document document;
    document.SetObject();
    Boolean(document,"enabled",timing.enabled);
    Boolean(document,"counter_saturated",timing.counter_saturated);
    Integer(document,"clock_failures",timing.clock_failures);
    Integer(document,"backward_samples",timing.backward_samples);
    for(bool last:{false,true}) {
        Value rows(rapidjson::kArrayType);
        const auto& counters=last?timing.last_step:timing.total;
        for(std::size_t i=0;i<counters.size();++i) {
            const auto& counter=counters[i];
            Document row;
            row.SetObject();
            String(row,"stage",vehicle_dynamics::StepStageNames[i]);
            Integer(row,"calls",counter.calls);
            Integer(row,"failures",counter.failures);
            Integer(row,"valid_samples",counter.valid_samples);
            Integer(row,"wall_ns",counter.wall_ns);
            Integer(row,"maximum_ns",counter.maximum_ns);
            Value value;
            value.CopyFrom(row,document.GetAllocator());
            rows.PushBack(value,document.GetAllocator());
        }
        document.AddMember(rapidjson::StringRef(last?"last_attempt":"total"),rows,document.GetAllocator());
    }
    return document;
}
}
records::RecordFile WriteSummary(const std::filesystem::path& root,const Config& config,const Horizon& horizon,
    const Forecast& forecast,const Result& result) {
    using namespace output;
    Document document;
    document.SetObject();
    String(document,"schema","robo_dyna.vehicle_run_summary.v1");
    String(document,"status",Name(result.loop.kind));
    String(document,"physical_profile",PhysicalProfileName(config.physical_profile));
    if(forecast.joint_count) Integer(document,"selected_joints",forecast.joint_count);
    String(document,"reason",result.loop.reason);
    String(document,"scope","observed accepted endpoints and host call timings; no restart or total-energy claim");
    Boolean(document,"session_initialized",result.session_initialized);
    Boolean(document,"valid_archive_manifest",result.loop.valid_manifest);
    Number(document,"startup_wall_s",result.startup_wall_s);
    Number(document,"requested_duration_s",config.duration_s);
    Number(document,"fixed_dt_s",config.fixed_dt_s);
    Integer(document,"planned_intervals",horizon.intervals);
    Integer(document,"planned_samples",config.samples);
    Integer(document,"complete_host_upper_bound",forecast.complete_host_bytes);
    Integer(document,"complete_archive_upper_bound",forecast.complete_archive_bytes);
    Integer(document,"host_cap",forecast.caps.host_bytes);
    Integer(document,"archive_cap",forecast.caps.archive_bytes);
    Boolean(document,"conditional_allowance_used",forecast.caps.expanded);
    if(result.session_initialized) {
        const auto& progress=result.loop.progress;
        Integer(document,"accepted_intervals",progress.accepted.epoch);
        Number(document,"actual_completed_time_s",progress.accepted.time_s);
        Number(document,"elapsed_after_startup_s",progress.elapsed_s);
        Number(document,"accepted_intervals_per_second",progress.accepted_intervals_per_second);
        Number(document,"successful_prepare_wall_s",progress.timing.step_s);
        Number(document,"successful_commit_wall_s",progress.timing.commit_s);
        Number(document,"successful_archive_wall_s",progress.timing.archive_s);
        Number(document,"successful_accepted_capture_wall_s",progress.timing.capture_s);
        const auto& contact=progress.contact;
        auto mechanics = MechanicsDocument(progress.mechanics);
        Value mechanics_value;
        mechanics_value.CopyFrom(mechanics,document.GetAllocator());
        document.AddMember("accepted_mechanics",mechanics_value,document.GetAllocator());
        Boolean(document,"contact_observations_available",contact.available);
        if(contact.available) {
            Number(document,"peak_observed_force_n",contact.peak_observed_force_n);
            Number(document,"peak_observed_penetration_m",contact.peak_observed_penetration_m);
            Number(document,"peak_observed_same_mask_potential_j",contact.peak_observed_potential_j);
            Number(document,"reported_signed_drift_work_sum_j",contact.reported_drift_work_sum_j);
            Number(document,"last_same_mask_potential_j",contact.last_same_mask_potential_j);
            Number(document,"last_removed_potential_j",contact.last_removed_potential_j);
            Integer(document,"last_accepted_active_parents",contact.accepted_active_parents);
            Integer(document,"last_proposed_active_parents",contact.proposed_active_parents);
        }
    }
    if(result.rejected_step_limit_s) {
        Number(document,"rejected_step_limit_s",*result.rejected_step_limit_s);
        String(document,"recovery","new run from original source with an explicitly selected smaller fixed timestep");
    }
    if(result.rejected_contact_status) Integer(document,"rejected_contact_status",static_cast<unsigned>(*result.rejected_contact_status));
    if(result.rejected_node!=UINT32_MAX) Integer(document,"rejected_physical_node",result.rejected_node);
    if(result.rejected_parent!=UINT32_MAX) Integer(document,"rejected_contact_parent",result.rejected_parent);
    if(result.archive_manifest) {
        String(document,"archive_manifest_file","archive/"+result.archive_manifest->file);
        String(document,"archive_manifest_sha256",result.archive_manifest->sha256);
    }
    if(result.viewer_input) {
        String(document,"viewer_input_file",result.viewer_input->file);
        String(document,"viewer_input_sha256",result.viewer_input->sha256);
    }
    if(!result.viewer_input_error.empty()) String(document,"viewer_input_error",result.viewer_input_error);
    auto timing=Timings(result.mechanics_timing);
    Value value;
    value.CopyFrom(timing,document.GetAllocator());
    document.AddMember("mechanics_stage_timing",value,document.GetAllocator());
    return physical_run::WriteDocument(root,"run-summary.json",document,SummaryByteCap);
}
} // namespace crash::cases::vehicle_run::detail
