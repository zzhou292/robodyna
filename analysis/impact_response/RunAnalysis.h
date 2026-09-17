#pragma once

#include "Report.h"
#include <filesystem>

namespace crash::analysis::impact_response {

struct AnalysisInput {
    std::filesystem::path viewer_input;
    std::string viewer_input_sha256;
    std::filesystem::path run_summary;
    std::string run_summary_sha256;
    std::filesystem::path connectivity_report;
    std::string connectivity_report_sha256;
};

output::Document Analyze(const AnalysisInput&, AnalysisLimits = {});

// Compact create-only report publication. The destination must be outside the
// immutable viewer/run directory and have an existing real parent directory.
void WriteReport(const std::filesystem::path& viewer_input,
    const std::filesystem::path& destination, const output::Document&,
    AnalysisLimits = {});

}  // namespace crash::analysis::impact_response
