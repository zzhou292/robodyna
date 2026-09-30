#include "Execution.h"
#include "output/source_assembly/SourceAssemblyWallArtifacts.h"
#include "output/source_assembly/QephSpinTrace.h"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace crash::cases::source_assembly_wall {
std::string FailureText(const source_assembly_dynamics::Report& report) {
    std::ostringstream out;
    out<<std::setprecision(17)<<report.message<<" parent="<<report.source_parent;
    if(report.node!=SIZE_MAX)out<<" node="<<report.node;
    out<<" measured="<<report.measured<<" limit="<<report.limit;
    return out.str();
}
int Execute(source_assembly_dynamics::SourceAssemblyWallCase& run,
            const output::assembly::WallArchiveRequest& request,const std::string& directory,output::assembly::QephSpinTrace* spin_trace) {
    output::assembly::SourceAssemblyWallArtifacts archive(directory,run,request);
    try {
        archive.WriteFrame(run);
        std::uint64_t last_saved=0;
        const auto started=std::chrono::steady_clock::now();
        const auto elapsed=[&] {return std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();};
        for(std::uint64_t step=1;step<=request.steps;++step) {
            const auto base=run.owner()->accepted();
            const auto result=run.Step();
            if(!result) {
                const auto reason=FailureText(result);
                if(!base.epoch||result.status==source_assembly_dynamics::Status::DeviceFailure)
                    throw std::runtime_error(reason);
                if(last_saved!=base.epoch)archive.WriteFrame(run);
                if(spin_trace)spin_trace->Finish(run,false,reason);
                archive.FinishPrefix(run,elapsed(),reason);
                std::cerr<<"Accepted assembly prefix through epoch "<<base.epoch<<": "<<reason<<'\n';
                return 2;
            }
            archive.RecordInterval(base,run);
            if(spin_trace)spin_trace->RecordInterval(run);
            if(step%request.frame_every==0||step==request.steps) {
                archive.WriteFrame(run);last_saved=step;
            }
            if(step%256==0||step==request.steps) {
                const auto& d=*run.diagnostics();const auto contact=run.accepted_contact();
                std::cout<<std::setprecision(10)<<"accepted "<<step<<'/'<<request.steps<<" t="<<d.stamp.time
                    <<" s wall_force="<<contact.diagnostics->resultant.value<<" N max_plastic_strain="
                    <<d.maximum_plastic_strain<<" plastic_work="<<d.cumulative_plastic_work
                    <<" J elapsed="<<elapsed()<<" s\n"<<std::flush;
            }
        }
        if(spin_trace)spin_trace->Finish(run,true,{});
        archive.Finish(run,elapsed());
        std::cout<<"Completed accepted assembly archive: "<<directory<<'\n';
        return 0;
    }catch(const std::exception& error) {archive.Fail(error.what());throw;}
}
}
