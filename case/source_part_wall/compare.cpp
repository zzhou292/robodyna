#include "output/SourcePartWallComparison.h"
#include <iostream>
int main(int argc,char** argv) {
    try {
        crash::output::Require(argc==5,"usage: robo_dyna_source_part_wall_compare H_DIR H2_DIR H4_DIR NEW_JSON");
        auto result=crash::output::CompareSourcePartWall({argv[1],argv[2],argv[3]});
        crash::output::WriteJson(argv[4],result.report);
        std::cout<<result.report["status"].GetString()<<'\n';return result.exit_code;
    } catch(const std::exception& e) {
        std::cerr<<"Source wall comparison incomplete: "<<e.what()<<'\n';return 1;
    }
}
