#include "RunAnalysis.h"
#include <iostream>

int main(int argc, char** argv) {
    try {
        using namespace crash::analysis::impact_response;
        crash::output::Require(argc == 8,
            "usage: robo_dyna_impact_response_analysis "
            "VIEWER_INPUT VIEWER_SHA256 RUN_SUMMARY RUN_SUMMARY_SHA256 "
            "CONNECTIVITY_REPORT CONNECTIVITY_SHA256 NEW_REPORT_JSON");
        const AnalysisInput input{
            argv[1], argv[2], argv[3], argv[4], argv[5], argv[6]};
        auto report = Analyze(input);
        WriteReport(input.viewer_input, argv[7], report);
        std::cout << "Published authenticated saved-response analysis: "
                  << std::filesystem::absolute(argv[7]) << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Impact response analysis incomplete: "
                  << error.what() << '\n';
        return 1;
    }
}
