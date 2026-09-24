#include "Options.h"
#include "../Run.h"
#include "../diagnostics/FailureRun.h"
#include "../SampledShellPlasticity.h"
#include "../SelfContactSummary.h"
#include "../contact_diagnostics/Document.h"
#include "../source/OriginalYaris.h"
#include <iomanip>
#include <iostream>
#include <utility>
namespace run=crash::cases::vehicle_run;
int main(int argc,char** argv) {
    if(argc==2 && std::string(argv[1])=="--help") {
        std::cout<<run::cli::Usage()<<'\n';
        return 0;
    }
    try {
        const auto options=run::cli::Parse(argc,argv);
        if(!options.forecast_only &&
            (std::filesystem::symlink_status(options.output).type()!=std::filesystem::file_type::directory ||
             !std::filesystem::is_empty(options.output)))
            throw std::invalid_argument("Output must be a real pre-created empty directory");
        std::filesystem::path failure_destination;
        if(!options.failure_output.empty()) {
            const auto& paths=options.source;
            failure_destination=run::diagnostics::CheckDestination(options.failure_output,options.output,
                {paths.canonical,paths.scope,paths.member,paths.declarations,paths.glass_resolution,
                 paths.type13,paths.auxiliary_member,paths.original_wall_member,paths.wall_manifest,
                 paths.self_contact_combine_member});
        }
        auto settings=crash::cases::vehicle_wall::LoadedWallSettings();
        settings.requested_duration_s=options.config.duration_s;
        settings.leading_gap_m=options.gap_m;
        if(options.wall_stiffness_n_m3) settings.stiffness_per_area=*options.wall_stiffness_n_m3;
        if(options.penetration_limit_m) settings.maximum_penetration_m=*options.penetration_limit_m;
        std::cout<<std::setprecision(17)<<"Preparing pinned original source using existing bounded factories"<<std::endl;
        const auto source=run::PrepareOriginalYaris(options.source,settings,
            options.config.physical_profile,options.config.contact_profile);
        run::records::Identity identity;
        identity.run=options.run_id;
        identity.topology=0x5941524953ULL;
        const auto prepared=run::PreparedRun::Prepare(source.setup,source.joints,options.config,identity,source.self_contact);
        const auto& forecast=prepared.forecast();
        const auto reservation=failure_destination.empty()
            ? run::diagnostics::Reservation{forecast.complete_host_bytes,forecast.complete_archive_bytes}
            : run::diagnostics::Preflight(forecast);
        std::cout<<"forecast physical_profile="<<run::PhysicalProfileName(options.config.physical_profile)
                 <<" contact_profile="<<run::ContactProfileName(options.config.contact_profile)
                 <<" host_bytes="<<reservation.host_bytes
                 <<" device_bytes="<<forecast.contact.device_bytes
                 <<" archive_bytes="<<reservation.archive_bytes
                 <<" intervals="<<prepared.horizon().intervals
                 <<" conditional_allowance="<<forecast.caps.expanded;
        if (options.config.self_contact_cuda_facet_filters)
            std::cout<<" self_contact_cuda_facet_filters_requested=1";
        std::cout<<std::endl;
        if(options.forecast_only) return 0;
        run::Control control;
        control.maximum_accepted_intervals=options.diagnostic_intervals;
        control.maximum_elapsed_s=options.maximum_elapsed_s;
        if(!options.stop_file.empty()) {
            control.stop_requested=[path=options.stop_file] {return std::filesystem::exists(path);};
        }
        control.progress=[](const run::Progress& value) {
            std::cout<<"accepted="<<value.accepted.epoch<<" time_s="<<value.accepted.time_s
                     <<" elapsed_s="<<value.elapsed_s<<" intervals_per_s="<<value.accepted_intervals_per_second
                     <<" contact_available="<<value.contact.available
                     <<" observed_peak_force_n="<<value.contact.peak_observed_force_n
                     <<" observed_peak_penetration_m="<<value.contact.peak_observed_penetration_m
                     <<" mechanics_available="<<value.mechanics.available
                     <<" observed_peak_translation_departure_m="<<value.mechanics.motion.translation_departure_m.peak
                     <<" observed_peak_velocity_departure_m_s="<<value.mechanics.motion.velocity_departure_m_s.peak
                     <<" observed_peak_spin_component_rad_s="<<value.mechanics.motion.spin_component_rad_s.peak
                     <<" solid_reported_plastic_work_sum_j="<<value.mechanics.solids.metal_plastic_work.work.accepted_increment_sum_j
                     <<" beam18_observations_available="<<(value.mechanics.available && value.mechanics.has_beam18)
                     <<" beam18_reported_plastic_work_sum_j="<<value.mechanics.beam18.plastic_work.work.accepted_increment_sum_j
                     <<" last_prepare_wall_s="<<value.mechanics_timing.last_step[0].wall_ns*1e-9;
            if(value.self_contact.available)
                std::cout<<" self_contact_events="<<value.self_contact.last_event_count
                         <<" self_contact_policy_outcomes="<<value.self_contact.last_policy_outcomes
                         <<" self_contact_policy_digest="<<value.self_contact.last_policy_digest;
            run::detail::WriteSelfContactWorkProgress(std::cout, value.self_contact);
            run::detail::WriteSampledShellPlasticityProgress(std::cout, value.sampled_shell_plasticity);
            std::cout << std::endl;
        };
        run::Result result;
        std::optional<run::diagnostics::FailureReport> failure_report;
        if(failure_destination.empty()) result=prepared.Execute(options.output,control);
        else {
            auto observed=run::diagnostics::Execute(prepared,options.output,control,failure_destination);
            result=std::move(observed.run);
            failure_report=std::move(observed.diagnostic);
        }
        std::cout<<"finished session_initialized="<<result.session_initialized<<" accepted="<<result.loop.progress.accepted.epoch
                 <<" actual_time_s="<<result.loop.progress.accepted.time_s
                 <<" valid_prefix="<<result.loop.valid_manifest<<" reason="<<result.loop.reason;
        run::detail::WriteSelfContactWorkProgress(std::cout, result.loop.progress.self_contact);
        run::detail::WriteSampledShellPlasticityProgress(std::cout, result.loop.progress.sampled_shell_plasticity);
        std::cout << std::endl;
        if(result.loop.kind==run::StopKind::PhysicsRejected && result.last_contact_attempt.enabled) {
            std::cout<<"self_contact_last_attempt";
            run::contact_diagnostics::WriteProgress(std::cout,result.last_contact_attempt);
            std::cout<<std::endl;
        }
        if(result.viewer_input) std::cout<<"viewer_descriptor="<<(options.output/result.viewer_input->file)
                                      <<" sha256="<<result.viewer_input->sha256<<std::endl;
        if(!result.summary_error.empty()) std::cerr<<"Summary error: "<<result.summary_error<<'\n';
        if(!result.viewer_input_error.empty()) std::cerr<<"Viewer receipt error: "<<result.viewer_input_error<<'\n';
        if(failure_report) {
            std::cout<<"self_contact_failure_diagnostic="
                     <<run::diagnostics::FailureStatusName(failure_report->status);
            if(!failure_report->manifest.empty())
                std::cout<<" manifest="<<failure_report->manifest<<" sha256="<<failure_report->sha256;
            std::cout<<std::endl;
            if(!failure_report->error.empty())
                std::cerr<<"Failure diagnostic error: "<<failure_report->error<<'\n';
        }
        if(result.loop.kind==run::StopKind::StartupFailure) return 1;
        if(!result.loop.valid_manifest || !result.summary || !result.viewer_input) return 3;
        if(failure_report && (failure_report->status==run::diagnostics::FailureStatus::CaptureIncomplete ||
            failure_report->status==run::diagnostics::FailureStatus::ExportFailed)) return 3;
        return result.loop.kind==run::StopKind::Completed?0:2;
    } catch(const std::exception& error) {
        std::cerr<<error.what()<<'\n'<<run::cli::Usage()<<'\n';
        return 1;
    }
}
