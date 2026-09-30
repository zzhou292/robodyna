#include "FailureRun.h"
#include "output/ArtifactIO.h"
#include <algorithm>

namespace crash::cases::vehicle_run::diagnostics {
namespace {
namespace fs = std::filesystem;
bool Within(const fs::path& child, const fs::path& parent) {
    return std::mismatch(parent.begin(), parent.end(), child.begin(), child.end()).first == parent.end();
}
}
std::filesystem::path CheckDestination(const fs::path& requested, const fs::path& run,
    const std::vector<fs::path>& protected_inputs) {
    output::Require(!requested.empty() && !run.empty(), "Failure diagnostics require explicit run and companion paths");
    const auto absolute = fs::absolute(requested).lexically_normal();
    output::Require(!absolute.filename().empty() &&
        fs::symlink_status(absolute).type() == fs::file_type::not_found &&
        fs::is_directory(absolute.parent_path()),
        "Failure companion must be absent with an existing directory parent");
    const auto destination = fs::canonical(absolute.parent_path()) / absolute.filename();
    output::Require(destination == absolute, "Failure companion parent must not contain symlink aliases");
    const auto canonical_run = fs::weakly_canonical(fs::absolute(run));
    output::Require(!Within(destination, canonical_run) && !Within(canonical_run, destination),
        "Failure companion must be separate from the authenticated run");
    for (const auto& input : protected_inputs) {
        output::Require(!input.empty(), "Protected diagnostic input path is empty");
        const auto canonical_input = fs::weakly_canonical(fs::absolute(input));
        output::Require(!Within(destination, canonical_input) && !Within(canonical_input, destination),
            "Failure companion overlaps a protected source input");
    }
    return destination;
}
const char* FailureStatusName(FailureStatus status) noexcept {
    switch (status) {
        case FailureStatus::NoRejection: return "no_rejection";
        case FailureStatus::Captured: return "captured";
        case FailureStatus::RejectionWithoutPair: return "rejection_without_authenticated_pair";
        case FailureStatus::CaptureIncomplete: return "capture_incomplete";
        case FailureStatus::ExportFailed: return "export_failed";
    }
    return "invalid_diagnostic_status";
}
} // namespace crash::cases::vehicle_run::diagnostics
