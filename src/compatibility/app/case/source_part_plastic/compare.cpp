#include "output/SourcePartPlasticComparison.h"
#include <iostream>

int main(int argc,char** argv) {
    try {
        crash::output::Require(argc==4,"usage: robo_dyna_source_part_plastic_compare H_DIR H2_DIR NEW_JSON");
        auto report=crash::output::CompareSourcePartPlastic({argv[1],argv[2]});
        crash::output::WriteJson(argv[3],report);
        std::cout<<report["status"].GetString()<<'\n';return 0;
    } catch(const std::exception& error) {
        std::cerr<<"Plastic sensitivity report incomplete: "<<error.what()<<'\n';return 1;
    }
}
