#include "Pilot.h"
#include <charconv>
#include <iostream>
#include <string_view>

namespace {
std::uint64_t Count(const char* input,std::uint64_t maximum) {
    const std::string_view value(input);std::uint64_t count=0;
    const auto parsed=std::from_chars(value.data(),value.data()+value.size(),count);
    if(parsed.ec!=std::errc{}||parsed.ptr!=value.data()+value.size()||!count||count>maximum)
        throw std::invalid_argument("Expected a positive count within the declared limit");
    return count;
}
}
int main(int argc,char** argv) {
    namespace wall=crash::cases::source_assembly_wall;
    try {
        if(argc!=6&&argc!=7)throw std::invalid_argument(
            "usage: robo_dyna_source_assembly_wall INVENTORY WALL STEPS FRAME_EVERY NEW_DIR [REFINEMENT_1_2_4]");
        crash::output::assembly::WallArchiveRequest request;
        request.steps=Count(argv[3],1<<20);request.frame_every=unsigned(Count(argv[4],request.steps));
        request.run_id=0x53415752554e31ULL;request.topology_id=0x534157544f5031ULL;request.asset_id=0x53415741535331ULL;
        const unsigned refinement=argc==7?unsigned(Count(argv[6],4)):1;
        crash::cases::source_assembly_dynamics::SourceAssemblyWallCase run;
        wall::InitializePilot(run,argv[1],argv[2],request,refinement);
        return wall::Execute(run,request,argv[5]);
    }catch(const std::exception& error) {
        std::cerr<<"Assembly output incomplete: "<<error.what()<<'\n';return 1;
    }
}
