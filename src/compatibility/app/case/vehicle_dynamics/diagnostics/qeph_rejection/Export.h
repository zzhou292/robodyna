#pragma once
#include "Capture.h"
#include "output/physical_run/Metadata.h"
#include <optional>
namespace crash::cases::vehicle_dynamics::diagnostics::qeph_rejection {
enum class ExportStatus { NoRejectedCandidate, Captured, CaptureIncomplete, ExportFailed };
const char* StatusName(ExportStatus) noexcept;
struct ExportResult {
    ExportStatus status = ExportStatus::NoRejectedCandidate;
    std::array<char, 1024> error{};
    std::optional<output::full_shell::RecordFile> manifest;
};
// The supplied companion directory must not exist. This publishes diagnostic
// values after ordinary prefix closure; it never authorizes acceptance/restart.
ExportResult Export(const CaptureState&, const std::filesystem::path&) noexcept;
// Also contains any allocation needed to form the companion path.
ExportResult ExportForRun(const CaptureState&, const std::filesystem::path& run_root) noexcept;
output::Document ExportDocument(const ExportResult&);
}
