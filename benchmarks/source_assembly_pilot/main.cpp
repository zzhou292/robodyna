#include "ComparisonInput.h"
#include <iostream>
int main(int argc,char** argv) {
    try {
        using namespace crash::benchmarks::assembly_pilot;
        Require(argc==4||argc==5,"usage: robo_dyna_source_assembly_pilot_compare FINE_REFERENCE CANDIDATE [CANDIDATE2] NEW_REPORT_JSON");
        Require(!std::filesystem::exists(std::filesystem::symlink_status(argv[argc-1])),"Pilot report output must not exist");
        std::vector<std::filesystem::path> inputs;for(int i=1;i<argc-1;++i)inputs.emplace_back(argv[i]);
        const auto report=Compare(inputs);const auto bytes=Encode(report)+"\n";
        Require(bytes.size()<=16*1024*1024,"Pilot comparison report exceeds 16 MiB cap");
        crash::output::WriteBytes(argv[argc-1],bytes);std::cout<<"Reported physical-time pilot differences; no convergence admission\n";return 0;
    } catch(const std::exception& e) {std::cerr<<"Pilot comparison incomplete: "<<e.what()<<'\n';return 1;}
}
