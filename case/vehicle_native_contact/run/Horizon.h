#pragma once
#include "output/full_shell/FixedStepHorizon.h"
namespace crash::cases::vehicle_native_contact::run_detail {
// Output admission only. Source preparation still checks its one-million-step
// ceiling, authenticated contact lifetimes and physical/resource limits.
inline constexpr double MaximumOutputDurationSeconds = .1;
inline bool PlanOutputHorizon(double fixed_dt, double duration,
                              std::uint64_t& intervals) noexcept {
    return duration <= MaximumOutputDurationSeconds &&
        output::full_shell::PlanFixedStepHorizon(fixed_dt, duration, intervals);
}
} // namespace crash::cases::vehicle_native_contact::run_detail
