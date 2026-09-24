#pragma once
#include "case/vehicle_run/Run.h"
#include <filesystem>
#include <string>
#include <vector>

namespace crash::cases::vehicle_run::diagnostics {
enum class FailureStatus {
    NoRejection, Captured, RejectionWithoutPair, CaptureIncomplete, ExportFailed
};
const char* FailureStatusName(FailureStatus) noexcept;
struct FailureReport {
    FailureStatus status = FailureStatus::NoRejection;
    std::filesystem::path manifest;
    std::string sha256, error;
};
struct Reservation { std::size_t host_bytes = 0, archive_bytes = 0; };
struct ObservedRun {
    Result run;
    Reservation reservation;
    FailureReport diagnostic;
};
// All diagnostics are a separate, create-only companion. These functions expose
// no live transaction, callback, initialization hook or publication authority.
std::filesystem::path CheckDestination(const std::filesystem::path& requested,
    const std::filesystem::path& run,
    const std::vector<std::filesystem::path>& protected_inputs = {});
Reservation Preflight(const Forecast&);
ObservedRun Execute(const PreparedRun&, const std::filesystem::path& run,
    const Control&, const std::filesystem::path& failure_destination);
} // namespace crash::cases::vehicle_run::diagnostics
