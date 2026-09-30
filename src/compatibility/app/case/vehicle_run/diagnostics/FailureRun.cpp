#include "FailureRun.h"
#include "Publication.h"
#include "case/vehicle_run/tests/observed/RunAccess.h"
#include <utility>

namespace crash::cases::vehicle_run::diagnostics {
Reservation Preflight(const Forecast& forecast) {
    const auto value = observed::Preflight(forecast);
    return {value.host_bytes, value.archive_bytes};
}
ObservedRun Execute(const PreparedRun& prepared, const std::filesystem::path& run,
    const Control& control, const std::filesystem::path& failure_destination) {
    const auto destination = CheckDestination(failure_destination, run);
    Preflight(prepared.forecast());
    auto execution = observed::RunAccess::Execute(prepared, run, control);
    ObservedRun result;
    result.reservation = {execution.reservation.host_bytes, execution.reservation.archive_bytes};
    result.run = std::move(execution.run);
    result.diagnostic = detail::PublishFailure(result.run, *execution.failure, destination);
    return result;
}
} // namespace crash::cases::vehicle_run::diagnostics
