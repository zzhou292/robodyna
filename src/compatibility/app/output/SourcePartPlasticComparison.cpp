#include "SourcePartPlasticComparison.h"
#include "SourcePartPlasticComparisonInput.h"

namespace crash::output {
namespace {
namespace pc=plastic_comparison;
namespace wc=wall_comparison;

Value Bracket(Document& report,const wc::EventBracket& event) {
    Value out(rapidjson::kObjectType);auto& a=report.GetAllocator();
    out.AddMember("observed",event.observed,a);
    out.AddMember("lower_s",event.lower,a);out.AddMember("upper_s",event.upper,a);
    out.AddMember("certificate_uncertainty_s",event.uncertainty,a);return out;
}

Value RunSummary(Document& report,const pc::Run& run) {
    Value out(rapidjson::kObjectType);auto& a=report.GetAllocator();
    const auto& info=*run.replay.info();const auto& o=run.observed;const auto& e=o.events;
    out.AddMember("directory",Value(run.directory.string().c_str(),a),a);
    out.AddMember("manifest_sha256",Value(run.manifest_hash.c_str(),a),a);
    out.AddMember("fixed_dt_s",wc::Real(run.configuration,"fixed_dt_s"),a);
    out.AddMember("rate_filter_alpha",wc::Real(run.configuration,"rate_filter_alpha"),a);
    out.AddMember("accepted_epoch",info.final_epoch,a);out.AddMember("accepted_time_s",info.final_time,a);
    out.AddMember("saved_frames",std::uint64_t(info.frame_count),a);
    out.AddMember("horizon_complete",info.horizon_complete,a);
    out.AddMember("stop_reason",Value(info.stop_reason.c_str(),a),a);
    out.AddMember("terminal_separated_rebound_observed",e.rebound.observed,a);
    out.AddMember("terminal_separation_start_epoch",e.terminal_separation_start,a);
    out.AddMember("first_contact",Bracket(report,e.onset),a);
    out.AddMember("terminal_rebound",Bracket(report,e.rebound),a);
    out.AddMember("contact_intervals",e.contact_intervals,a);
    out.AddMember("peak_reaction_N",e.peak_reaction,a);
    out.AddMember("peak_reaction_certificate_error_N",e.peak_reaction_error,a);
    out.AddMember("maximum_penetration_m",e.peak_penetration,a);
    out.AddMember("maximum_physical_energy_uncertainty_J",e.maximum_energy_uncertainty,a);
    out.AddMember("maximum_absolute_energy_residual_J",o.maximum_absolute_energy_residual,a);
    out.AddMember("maximum_declared_energy_budget_fraction",o.maximum_energy_bound_fraction,a);
    out.AddMember("final_carried_COM_velocity_x_m_per_s",o.final_raw_com_velocity,a);
    out.AddMember("final_maximum_chord_change_m",o.final_chord_change,a);
    out.AddMember("final_maximum_plastic_strain",o.final_maximum_plastic_strain,a);
    out.AddMember("final_mean_plastic_strain",o.final_mean_plastic_strain,a);
    out.AddMember("final_cumulative_plastic_work_J",o.final_plastic_work,a);
    out.AddMember("final_yielded_points",o.final_yielded_points,a);
    out.AddMember("final_yielded_parents",o.final_yielded_parents,a);
    out.AddMember("accumulated_plastic_history_observed",o.final_maximum_plastic_strain>0&&o.final_plastic_work>0,a);
    const auto numerical=pc::ReadNumericalResponse(run.final,run.initial_kinetic);
    Value work(rapidjson::kObjectType);
    work.AddMember("hourglass_viscous_work_J",numerical.hourglass_viscous_work,a);
    work.AddMember("hourglass_viscous_fraction_of_initial_kinetic",numerical.viscous_fraction_of_initial_kinetic,a);
    work.AddMember("carried_rotation_total_J",numerical.carried_rotation_total,a);
    work.AddMember("carried_rotation_physical_isotropic_J",numerical.carried_rotation_physical,a);
    work.AddMember("carried_rotation_added_isotropic_J",numerical.carried_rotation_added,a);
    work.AddMember("carried_velocity_time_s",numerical.carried_velocity_time,a);
    out.AddMember("final_numerical_response",work,a);
    return out;
}
}

Document CompareSourcePartPlastic(const std::array<std::filesystem::path,2>& directories) {
    std::array<pc::Run,2> runs;
    for(unsigned i=0;i<2;++i)runs[i].Open(directories[i]);
    const auto schedule=pc::MatchConfiguration(runs[0].configuration,runs[1].configuration);
    for(const char* file:{"placed-wall-placement.json","placed-wall.mesh.json","original-canonical-wall.manifest.json"})
        Require(ReadBounded(directories[0]/file,32*1024*1024)==ReadBounded(directories[1]/file,32*1024*1024),
            "Plastic runs changed the actual placed wall or original wall source");
    const auto count=pc::SharedStrideCount(schedule,runs[0].replay.info()->final_epoch,runs[1].replay.info()->final_epoch);
    const double last_common_time=count*schedule.cadence;
    const bool full=runs[0].replay.info()->horizon_complete&&runs[1].replay.info()->horizon_complete;
    if(full)Require(Bits(last_common_time)==Bits(schedule.requested_horizon),
        "Full plastic horizons lack their declared final common sample");

    Document report;report.SetObject();auto& a=report.GetAllocator();
    pc::Differences maximum{},last{},maximum_time{};
    std::array<long double,pc::Channels> sum_square{};
    Value samples(rapidjson::kArrayType);
    for(std::uint64_t k=0;k<=count;++k) {
        const auto coarse=runs[0].FrameAt(k*schedule.coarse_stride);
        const auto fine=runs[1].FrameAt(k*schedule.fine_stride);
        last=pc::Difference(coarse,fine);const double time=k*schedule.cadence;
        Require(Bits(wc::Real(coarse,"accepted_time_s"))==Bits(time),"Shared plastic sample has the wrong cadence time");
        for(unsigned c=0;c<pc::Channels;++c) {
            if(last[c]>maximum[c]) {maximum[c]=last[c];maximum_time[c]=time;}
            sum_square[c]+=static_cast<long double>(last[c])*last[c];
        }
        Value row(rapidjson::kArrayType);row.PushBack(time,a);
        for(double difference:last)row.PushBack(difference,a);
        samples.PushBack(row,a);
    }
    Value channels(rapidjson::kArrayType),summaries(rapidjson::kArrayType);
    for(unsigned c=0;c<pc::Channels;++c) {
        Value channel(rapidjson::kObjectType);
        const double rms=static_cast<double>(std::sqrt(sum_square[c]/(count+1)));
        Require(std::isfinite(rms),"Nonfinite plastic sensitivity statistic");
        channel.AddMember("field",Value(pc::Names[c],a),a);
        channel.AddMember("maximum_absolute_difference",maximum[c],a);
        channel.AddMember("time_of_maximum_s",maximum_time[c],a);
        channel.AddMember("rms_of_sample_differences",rms,a);
        channel.AddMember("last_common_absolute_difference",last[c],a);channels.PushBack(channel,a);
    }
    for(const auto& run:runs)summaries.PushBack(RunSummary(report,run),a);
    String(report,"schema","robo_dyna.source_part_plastic_sensitivity.v1");
    String(report,"status","sensitivity_report");Boolean(report,"convergence_claimed",false);
    String(report,"scope","Two timestep resolutions of original Yaris part 2000157 against its placed mesh wall; source attachments unapplied; no full-vehicle or numerical convergence claim");
    String(report,"material_model",runs[0].replay.info()->material_model);
    String(report,"material_policy",runs[0].replay.info()->material_policy);
    String(report,"physical_configuration_sha256",Sha256(runs[0].physical_configuration));
    String(report,"configuration_comparison","Every physical and source field must match; only timestep/output identities and the timestep-derived rate filter alpha may differ");
    String(report,"sampling","Exact shared physical cadence, including startup; distinct first-step frames and unmatched prefix endpoints are omitted; no interpolation");
    String(report,"kinematic_statistic","Per-sample maximum nodal vector norm and sign-invariant quaternion rotation distance; velocity and angular velocity use synchronized endpoint fields");
    String(report,"plastic_statistic","Per-sample maximum difference over corresponding 94-by-3 point PLA and 94 cumulative parent work values; RMS is over these sampled maxima");
    String(report,"energy_policy","Ksync + native EINT/EVIS + contact potential - K0; point plastic work is a separate diagnostic already represented in native work");
    String(report,"numerical_response_policy","Native hourglass viscous work excludes recoverable stabilization work; native added-isotropic rotational energy is reported separately at each run's carried velocity time. Small balance error and timestep sensitivity alone do not qualify spatial stabilization or added-inertia response");
    String(report,"unloading_scope","Accumulated plastic strain records irreversible material history. Separated rebound and changing chord lengths do not establish a stress-free final shape; elastic vibration may remain");
    String(report,"rebound_policy","All accepted intervals are inspected; terminal strictly-separated run of at least 128H, zero certified force/potential and negative carried COM upper bound; later recontact resets it");
    Boolean(report,"all_accepted_intervals_satisfy_declared_energy_bound",true);
    Boolean(report,"both_requested_horizons_complete",full);
    Integer(report,"common_samples",count+1);Number(report,"sample_cadence_s",schedule.cadence);
    Number(report,"requested_horizon_s",schedule.requested_horizon);Number(report,"last_common_time_s",last_common_time);
    Number(report,"required_terminal_separation_duration_s",wc::CommonStride*wc::BaseStep);
    Number(report,"native_total_mass_kg",runs[0].total_mass);Number(report,"initial_kinetic_J",runs[0].initial_kinetic);
    Number(report,"initial_velocity_x_m_per_s",runs[0].speed);
    String(report,"sample_columns","time_s followed by channels in their declared order");
    report.AddMember("channels",channels,a);report.AddMember("samples",samples,a);report.AddMember("runs",summaries,a);
    return report;
}
} // namespace crash::output
