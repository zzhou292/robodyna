#include "Q4ContactProfile.h"
#include <filesystem>
#include <iostream>
#include <stdexcept>

int main(int argc,char** argv) {
    try {
        if(argc!=5&&argc!=6)throw std::runtime_error("Usage: robo-dyna-contact-profile WALL FRAME EXPECTED-FRAME-SHA256 NEW-REPORT [--backend=scalar|--backend=rectangular]");
        auto backend=crash::profile::ContactProfileBackend::Scalar;
        if(argc==6) {
            const std::string argument=argv[5];
            if(argument=="--backend=rectangular")backend=crash::profile::ContactProfileBackend::Rectangular;
            else if(argument!="--backend=scalar")throw std::runtime_error("Unknown prescribed contact profile backend option");
        }
        if(std::filesystem::exists(argv[4]))throw std::runtime_error("Report must be new");
        const auto input=crash::profile::ReadContactProfileSource(argv[1],argv[2],argv[3]);
        const auto result=crash::profile::MeasureContactProfile(input.input,backend);
        crash::profile::WriteContactProfile(argv[4],input,result);
        std::cout<<"Prescribed CPU/CUDA contact profile passed; "<<argv[4]<<'\n';return 0;
    } catch(const std::exception& e) {std::cerr<<"Contact profile failed: "<<e.what()<<'\n';return 1;}
}
