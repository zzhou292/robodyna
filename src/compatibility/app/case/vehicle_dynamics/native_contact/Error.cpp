#include "Error.h"
#include <string>

namespace crash::cases::vehicle_dynamics::native_contact {
namespace {
const char* Name(Operation operation) {
    switch (operation) {
    case Operation::Adopt: return "Adopt native contact contribution";
    case Operation::Assemble: return "Assemble native accepted contact";
    case Operation::SealCandidate: return "Seal native contact candidate";
    }
    return "Native contact contribution";
}
std::string Message(Operation operation, const native::TransactionReport& report,
                    const native::TransactionDiagnostics& diagnostics) {
    auto result = std::string(Name(operation)) + ": " +
        (report.message ? report.message : "Native operation rejected") +
        " [status=" + std::to_string(int(report.status)) +
        " row=" + (report.row == SIZE_MAX ? "unavailable" : std::to_string(report.row)) +
        " occurrence=" + (report.occurrence == SIZE_MAX ? "unavailable" : std::to_string(report.occurrence)) +
        " selection_status=" + std::to_string(int(report.selection_status)) + "]";
    if (diagnostics.maintenance_failure_available) {
        const auto& failure = diagnostics.maintenance_failure;
        result += " [maintenance_operation=" + std::to_string(int(diagnostics.maintenance_operation)) +
            " search_status=" + std::to_string(int(failure.status)) +
            " search_row=" + (failure.row_available ? std::to_string(failure.input_row) : "unavailable") + "]";
    }
    return result;
}
} // namespace
StageError::StageError(Operation operation, const native::TransactionReport& report,
                       const native::TransactionSourceInfo& source, const native::TransactionDiagnostics& diagnostics)
    : std::runtime_error(Message(operation, report, diagnostics)),
      failure_{operation, report.status, report.row, report.occurrence, report.selection_status, source, diagnostics} {}
} // namespace crash::cases::vehicle_dynamics::native_contact
