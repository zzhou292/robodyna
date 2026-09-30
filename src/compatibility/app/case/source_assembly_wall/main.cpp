#include "Pilot.h"
#include "StepTimingReport.h"
#include "CliOptions.h"
#include "output/source_assembly/QephSpinTrace.h"
#include <iostream>

int main(int argc,char** argv) {
    namespace wall=crash::cases::source_assembly_wall;
    crash::cases::source_assembly_dynamics::SourceAssemblyWallCase run;
    std::string timing_path;int status=1;
    try {
        const auto options=wall::ParseOptions(argc,argv);timing_path=options.timing_path;
        if(!timing_path.empty())wall::CheckStepTimingPath(timing_path,options.archive);
        if(!options.spin_path.empty()) {
            wall::CheckStepTimingPath(options.spin_path,options.archive);
            crash::output::assembly::PlanQephSpinTrace(options.steps,options.spin_every);
            if(std::filesystem::exists(options.spin_path)||(!timing_path.empty()&&
               std::filesystem::weakly_canonical(options.spin_path)==std::filesystem::weakly_canonical(timing_path)))
                throw std::invalid_argument("Spin output must be new and distinct from timing output");
        }
        crash::output::assembly::WallArchiveRequest request;
        request.steps=options.steps;request.frame_every=options.frame_every;
        request.run_id=0x53415752554e31ULL;request.topology_id=0x534157544f5031ULL;request.asset_id=0x53415741535331ULL;
        wall::InitializePilot(run,options.inventory,options.wall,request,options.pilot);
        std::unique_ptr<crash::output::assembly::QephSpinTrace> spin;
        if(!options.spin_path.empty())spin=std::make_unique<crash::output::assembly::QephSpinTrace>(options.spin_path,run,options.steps,options.spin_every);
        status=wall::Execute(run,request,options.archive,spin.get());
    }catch(const std::exception& error) {
        std::cerr<<"Assembly output incomplete: "<<error.what()<<'\n';
    }
    if(!timing_path.empty()&&run.initialized()) {
        try {wall::WriteStepTiming(timing_path,run.timing(),status);}
        catch(const std::exception& error) {std::cerr<<"Stage timing report failed: "<<error.what()<<'\n';}
    }
    return status;
}
