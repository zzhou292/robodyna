#include "ThinShellScreenReport.h"

#include <iostream>

int main(int argc,char** argv) {
    namespace io=crash::output;
    try {
        io::Require(argc==3,"Usage: robo-dyna-thin-shell-screen PROVENANCE-JSON NEW-REPORT-JSON");
        const std::filesystem::path output=argv[2];
        io::Require(!std::filesystem::exists(std::filesystem::symlink_status(output)),"Thin-shell output must be new");
        const auto parent=output.has_parent_path()?output.parent_path():std::filesystem::path(".");
        io::Require(std::filesystem::is_directory(parent),"Thin-shell output parent must exist");
        const auto bytes=io::ReadBounded(argv[1],1024*1024);
        io::Document provenance; provenance.Parse(bytes.c_str(),bytes.size());
        io::Require(!provenance.HasParseError()&&provenance.IsObject()&&!provenance.ObjectEmpty(),
                    "Thin-shell provenance must be a nonempty bounded JSON object");
        // Bind the precise caller inventory bytes as well as its readable data.
        io::Document binding; binding.SetObject(); io::String(binding,"sha256",io::Sha256(bytes));
        io::Integer(binding,"bytes",bytes.size()); io::Value inventory;
        inventory.CopyFrom(provenance,binding.GetAllocator()); binding.AddMember("inventory",inventory,binding.GetAllocator());
        const auto result=crash::reference::ScreenThinShellFixtures();
        io::WriteJson(output,crash::reference::ThinShellScreenReport(result,binding));
        std::cout<<"Thin-shell screen "<<(result.screen_passed?"passed":"rejected")
                 <<"; all six diagnostics saved; simulation_ready=false\n";
        return result.screen_passed?0:2;
    } catch(const std::exception& e) {
        std::cerr<<"Thin-shell report not completed: "<<e.what()<<'\n'; return 1;
    }
}
