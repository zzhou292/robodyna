#include "src/simulation/driver/native/Session.h"
#include <iostream>
#include <optional>

int main(int argc, char** argv) {
    namespace driver = robodyna::driver;
    namespace io = crash::output;
    std::optional<driver::Request> request;
    try {
        io::Require(argc == 7 && std::string(argv[1]) == "--mode" && std::string(argv[3]) == "--request" &&
                    std::string(argv[5]) == "--request-sha256", "Expected --mode plan|run --request FILE --request-sha256 SHA256");
        const std::string mode(argv[2]);
        io::Require(mode == "plan" || mode == "run", "Unsupported native driver mode");
        request = driver::ReadRequest(argv[4], argv[6]);
        io::Require(!std::filesystem::exists(request->report), "Native report destination already exists");
        const auto prepared = driver::Prepare(*request);
        auto report = driver::PlanDocument(*request, prepared, mode.c_str());
        const int code = mode == "run" ? driver::Execute(*request, prepared, report) : 0;
        io::Integer(report, "exit_code", static_cast<unsigned>(code));
        io::WriteJson(request->report, report);
        return code;
    } catch (const std::exception& error) {
        std::cerr << "robodyna native: " << error.what() << std::endl;
        if (request && !std::filesystem::exists(request->report)) {
            try {
                io::Document report;
                report.SetObject();
                io::String(report, "schema", "robodyna.native_vehicle_driver.v1");
                io::String(report, "status", "failed");
                io::String(report, "error", std::string(error.what()).substr(0, 4096));
                io::Integer(report, "exit_code", 1);
                io::Boolean(report, "horizon_complete", false);
                io::WriteJson(request->report, report);
            } catch (const std::exception& reporting) {
                std::cerr << "robodyna report: " << reporting.what() << std::endl;
            }
        }
        return 1;
    }
}
