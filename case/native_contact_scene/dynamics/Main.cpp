#include "Options.h"
#include <iomanip>
#include <iostream>
namespace scene=crash::cases::native_scene;
int main(int argc,char** argv) {
    if(argc==2&&std::string(argv[1])=="--help"){std::cout<<scene::cli::Usage()<<'\n';return 0;}
    try {
        auto options=scene::cli::Parse(argc,argv);
        crash::output::Require(std::filesystem::symlink_status(options.source_output).type()==std::filesystem::file_type::directory&&
            std::filesystem::is_empty(options.source_output),"Source output must be a real pre-created empty directory");
        if(!options.forecast_only)crash::output::Require(std::filesystem::symlink_status(options.output).type()==std::filesystem::file_type::directory&&
            std::filesystem::is_empty(options.output),"Run output must be a real pre-created empty directory");
        const auto declared=crash::modelio::native_scene::DeclaredSource::Read(options.source,options.source_sha256);
        const auto physical=scene::PhysicalSource::Prepare(declared,options.config.run_id);
        const auto contact=scene::ContactSource::Prepare(physical,{options.config.run_id,1,1});
        const auto archive=scene::ArchiveSource::Write(physical,options.source_output);
        const auto prepared=scene::PreparedNativeSceneRun::Prepare(contact,archive,options.config);
        const auto& f=prepared.forecast();
        std::cout<<std::setprecision(17)<<"forecast host_upper_bound_bytes="<<f.host_upper_bound
            <<" device_upper_bound_bytes="<<f.device_upper_bound<<" archive_bytes="<<f.archive.archive.archive.forecast_bytes
            <<" intervals="<<prepared.horizon().intervals<<" fixed_dt_s="<<prepared.horizon().fixed_dt_s<<std::endl;
        if(options.forecast_only){crash::output::WriteJson(options.source_output/"run-forecast.json",prepared.ForecastDocument());return 0;}
        if(!options.stop_file.empty())options.control.stop_requested=[path=options.stop_file]{return std::filesystem::exists(path);};
        options.control.progress=[](const crash::cases::vehicle_run::Progress& p){std::cout<<"accepted="<<p.accepted.epoch
            <<" time_s="<<p.accepted.time_s<<" elapsed_s="<<p.elapsed_s<<std::endl;};
        const auto result=prepared.Execute(options.output,options.control);
        std::cout<<"finished accepted="<<result.loop.progress.accepted.epoch<<" actual_time_s="<<result.loop.progress.accepted.time_s
            <<" valid_closed_archive="<<result.loop.valid_manifest<<" reason="<<result.loop.reason<<std::endl;
        if(!result.output_error.empty()){std::cerr<<result.output_error<<'\n';return 1;}
        if(result.loop.kind==crash::cases::vehicle_run::StopKind::Completed&&result.loop.valid_manifest&&result.viewer_input&&result.summary)return 0;
        return result.loop.valid_manifest&&result.viewer_input&&result.summary?2:1;
    }catch(const std::exception& e){std::cerr<<"native scene: "<<e.what()<<'\n';return 1;}
}
