#pragma once

#include "case/vehicle_run/Run.h"
#include "case/vehicle_startup/shell_execution/tests/self_contact/CandidateFailureFixture.h"

namespace crash::cases::vehicle_run::observed {

namespace fixture = vehicle_startup::shell_execution::self_contact_test;
struct Reservation {
    std::size_t host_bytes = 0;
    std::size_t archive_bytes = 0;
};
// Pure host admission; preserves configured profile caps and charges both the
// retained callback storage and its possible exported diagnostic companion.
Reservation Preflight(const Forecast&);

struct ObservedResult {
    Result run;
    Reservation reservation;
    std::unique_ptr<fixture::CandidateFailureFixture> failure;
};
class RunAccess {
  public:
    // Synchronous; capture uses this run's exact retained proof limits and may
    // be exported afterward. It never changes a result or authorizes a step.
    static ObservedResult Execute(const PreparedRun&, const std::filesystem::path&,
                                  const Control& = {});
};

} // namespace crash::cases::vehicle_run::observed
