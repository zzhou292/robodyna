#include "GuidedPlateCli.h"
#include <exception>
#include <iostream>

int main(int argc,char** argv) {
    try {
        return crash::case_data::ExecuteGuidedPlateCommand(crash::case_data::ParseGuidedPlateCommand(argc,argv));
    } catch(const std::exception& e) {
        std::cerr<<"Guided study incomplete: "<<e.what()<<'\n';return 1;
    }
}
