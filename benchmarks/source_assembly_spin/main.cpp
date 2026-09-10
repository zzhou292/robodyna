#include "Analysis.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <iostream>
int main(int argc,char** argv) {
    try {
        using namespace crash;output::Require(argc==4,"usage: robo_dyna_source_assembly_spin_analyze INVENTORY COMPLETED_TRACE NEW_REPORT_JSON");
        output::Require(!std::filesystem::exists(std::filesystem::symlink_status(argv[3])),"Spin analysis report must be new");
        const auto report=benchmarks::assembly_spin::Analyze(argv[1],argv[2]);rapidjson::StringBuffer bytes;
        rapidjson::Writer<rapidjson::StringBuffer> writer(bytes);output::Require(report.Accept(writer)&&bytes.GetSize()<32*1024*1024,"Spin analysis report exceeds its 32 MiB cap");
        output::WriteBytes(argv[3],std::string(bytes.GetString(),bytes.GetSize())+'\n');
        std::cout<<"Reported independent native recurrence and local spin diagnostics; no guard change authorized\n";return 0;
    }catch(const std::exception& e){std::cerr<<"Spin analysis incomplete: "<<e.what()<<'\n';return 1;}
}
