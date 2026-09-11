#include "Options.h"
#include "../Run.h"
#include "../source/OriginalYaris.h"
#include <iomanip>
#include <iostream>
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
        auto settings=crash::cases::vehicle_wall::LoadedWallSettings();
        settings.requested_duration_s=options.config.duration_s;
        settings.leading_gap_m=options.gap_m;
        if(options.wall_stiffness_n_m3) settings.stiffness_per_area=*options.wall_stiffness_n_m3;
        if(options.penetration_limit_m) settings.maximum_penetration_m=*options.penetration_limit_m;
        std::cout<<std::setprecision(17)<<"Preparing pinned original source using existing bounded factories"<<std::endl;
        const auto source=run::PrepareOriginalYaris(options.source,settings);
        run::records::Identity identity;
        identity.run=options.run_id;
        identity.topology=0x5941524953ULL;
        const auto prepared=run::PreparedRun::Prepare(source.setup,source.joints,options.config,identity);
        const auto& forecast=prepared.forecast();
        std::cout<<"forecast host_bytes="<<forecast.complete_host_bytes
                 <<" device_bytes="<<forecast.wall.device_bytes
                 <<" archive_bytes="<<forecast.complete_archive_bytes
                 <<" intervals="<<prepared.horizon().intervals
                 <<" conditional_allowance="<<forecast.caps.expanded<<std::endl;
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
                     <<" last_prepare_wall_s="<<value.mechanics_timing.last_step[0].wall_ns*1e-9<<std::endl;
        };
        const auto result=prepared.Execute(options.output,control);
        std::cout<<"finished session_initialized="<<result.session_initialized<<" accepted="<<result.loop.progress.accepted.epoch
                 <<" actual_time_s="<<result.loop.progress.accepted.time_s
                 <<" valid_prefix="<<result.loop.valid_manifest<<" reason="<<result.loop.reason<<std::endl;
        if(result.viewer_input) std::cout<<"viewer_descriptor="<<(options.output/result.viewer_input->file)
                                      <<" sha256="<<result.viewer_input->sha256<<std::endl;
        if(!result.summary_error.empty()) std::cerr<<"Summary error: "<<result.summary_error<<'\n';
        if(!result.viewer_input_error.empty()) std::cerr<<"Viewer receipt error: "<<result.viewer_input_error<<'\n';
        if(result.loop.kind==run::StopKind::StartupFailure) return 1;
        if(!result.loop.valid_manifest || !result.summary || !result.viewer_input) return 3;
        return result.loop.kind==run::StopKind::Completed?0:2;
    } catch(const std::exception& error) {
        std::cerr<<error.what()<<'\n'<<run::cli::Usage()<<'\n';
        return 1;
    }
}
