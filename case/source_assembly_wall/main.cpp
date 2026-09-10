#include "Pilot.h"
#include "StepTimingReport.h"
#include "CliOptions.h"
#include <iostream>

int main(int argc,char** argv) {
    namespace wall=crash::cases::source_assembly_wall;
    crash::cases::source_assembly_dynamics::SourceAssemblyWallCase run;
    std::string timing_path;int status=1;
    try {
        const auto options=wall::ParseOptions(argc,argv);timing_path=options.timing_path;
        if(!timing_path.empty())wall::CheckStepTimingPath(timing_path,options.archive);
        crash::output::assembly::WallArchiveRequest request;
        request.steps=options.steps;request.frame_every=options.frame_every;
        request.run_id=0x53415752554e31ULL;request.topology_id=0x534157544f5031ULL;request.asset_id=0x53415741535331ULL;
        wall::InitializePilot(run,options.inventory,options.wall,request,options.pilot);
        status=wall::Execute(run,request,options.archive);
    }catch(const std::exception& error) {
        std::cerr<<"Assembly output incomplete: "<<error.what()<<'\n';
    }
    if(!timing_path.empty()&&run.initialized()) {
        try {wall::WriteStepTiming(timing_path,run.timing(),status);}
        catch(const std::exception& error) {std::cerr<<"Stage timing report failed: "<<error.what()<<'\n';}
    }
    return status;
}
